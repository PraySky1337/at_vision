#include "rune_detector/rune_detector.hpp"
#include "rm_utils/logger/log.hpp"
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// std
#include <algorithm>
#include <fstream>
#include <numeric>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <unordered_map>
// third party
#include <angles/angles.h>
#include <opencv2/imgproc.hpp>
#include <vector>
// project
#include "rune_detector/types.hpp"
#include <rclcpp/rclcpp.hpp>

namespace fyt::rune {
bool inRect(const cv::Point2f& p, const cv::Rect2f& rect) {
    return (
        p.x >= rect.x && p.x <= rect.x + rect.width && p.y >= rect.y
        && p.y <= rect.y + rect.height);
}

/*处理图片*/
cv::Mat RuneDetector::processPictures(const cv::Mat& src) {
    // 二值化+开闭运算
    std::vector<cv::Mat> ch;
    cv::split(src, ch);               // B,G,R
    cv::Mat img;

    cv::Mat blue{ch[0](globalRoi)}, red{ch[2](globalRoi)};
    if (detect_color_ == EnemyColor::RED)
        cv::subtract(red, blue, img); // R - B

    else
        cv::subtract(blue, red, img); // B - R

    cv::threshold(img, arrowImg, arrow_threshold_, 255, cv::THRESH_BINARY);
    cv::threshold(img, targetImg, target_threshold_, 255, cv::THRESH_BINARY);
    cv::threshold(img, rCenterImg, rcenter_threshold_, 255, cv::THRESH_BINARY);

    return img;
}

/*裁剪*/
void resetRoi(cv::Rect2f& rect, int rows, int cols) {
    // 调整左上角点的坐标
    rect.x = rect.x < 0 ? 0 : rect.x >= cols ? cols - 1 : rect.x;
    rect.y = rect.y < 0 ? 0 : rect.y >= rows ? rows - 1 : rect.y;
    // 调整长宽
    rect.width  = rect.x + rect.width >= cols ? cols - rect.x - 1 : rect.width;
    rect.height = rect.y + rect.height >= rows ? rows - rect.y - 1 : rect.height;
    // 此时可能出现 width 或 height 小于 0 的情况，因此需要将其置为 0
    if (rect.width < 0) {
        rect.width = 0;
    }
    if (rect.height < 0) {
        rect.height = 0;
    }
}

/*求点到两个点拟合出直线的距离*/
float pointLineDistance(const cv::Point2f& p, const cv::Point2f& a, const cv::Point2f& b) {
    float A = b.y - a.y;
    float B = a.x - b.x;
    float C = b.x * a.y - a.x * b.y;
    return std::fabs(A * p.x + B * p.y + C) / std::sqrt(A * A + B * B);
}

/*求点到一条直线的距离*/
inline float pointLineDistance(const cv::Point2f& point, const cv::Vec4f& line) {
    float vx = line[0], vy = line[1];
    float x0 = line[2], y0 = line[3];
    float px = point.x, py = point.y;

    // 点到直线距离公式
    float distance = std::abs((x0 - px) * vy - (y0 - py) * vx);
    return distance;
}

/*设置流水灯的参数*/
void Arrow::set(const std::vector<Light>& lights, const cv::Point2f& roi) {
    std::vector<cv::Point2f> arrowPoints;
    double fillArea        = 0.0;
    double pointLineThresh = 0.0;
    std::for_each(lights.begin(), lights.end(), [&](const Light& l) {
        arrowPoints.insert(arrowPoints.end(), l.contour.begin(), l.contour.end());
        fillArea += l.contourArea;
        pointLineThresh += l.size.height / lights.size();
    });
    // 滤除距离较大的点
    contour.clear();
    cv::Vec4f line;
    cv::fitLine(arrowPoints, line, cv::DIST_L2, 0, 0.01, 0.01);
    for (const auto& point : arrowPoints) {
        if (pointLineDistance(point, line) < pointLineThresh) {
            contour.push_back(point);
        }
    }
    // 设置成员变量
    rotated = cv::minAreaRect(contour);

    center = rotated.center + roi;
    // 较长的一边为height，较短的一边为width
    size.height = rotated.size.height;
    size.width  = rotated.size.width;
    // RotatedRect::angle 范围为 -90~0. 这里根据长宽长度关系，将角度扩展到 -90~90
    if (size.height < size.width) {
        angle = rotated.angle;
        // 长的为 length
        std::swap(size.height, size.width);
    } else {
        angle = rotated.angle + 90;
    }
    ratio     = size.height / size.width;
    area      = size.height * size.width;
    fillratio = fillArea / area;
    return;
}

/*设置R标中心的参数*/
void CenterR::set(const Light& light) {
    this->light  = light;
    boundingRect = cv::boundingRect(light.contour);
    // 由于灯条角点和中心点已经设置过 roi，因此这里不需要重新设置
    center = light.center;
    return;
}

/*设置靶的参数*/
void Target::set(const Light& l) {
    // 由于中心点已经设置过 roi，因此这里不需要重新设置
    center  = l.center;
    contour = l.contour;
    return;
}

Light::Light(
    const std::vector<cv::Point>& cnt, const cv::Rect2f& globalRoi, const cv::Rect2f& localRoi)
    : contour(cnt)
    , contourArea(cv::contourArea(cnt))
    , rotated(cv::minAreaRect(cnt)) {
    // 长的为 height ，短的为 width
    size.width = rotated.size.width, size.height = rotated.size.height;
    if (size.width > size.height) {
        std::swap(size.width, size.height);
    }
    ratio     = size.height / size.width;
    center    = rotated.center;
    angle     = rotated.angle;
    area      = rotated.size.width * rotated.size.height;
    float len = cv::arcLength(contour, true);
    roundness = (4 * CV_PI * contourArea) / (len * len);

    center += localRoi.tl() + globalRoi.tl();
}

void RuneDetector::setGlobalRoi() {
    double widths[2];
    for (std::size_t i = 0; i < targets.size(); i++) {
        widths[i] = GLOBAL_ROI_LENGTH_RATIO * 2 * cv::norm(targets[i].center - rcenter.center);
    }
    double width;
    if (widths[1] == 0) {
        width = widths[0];
    } else {
        width = (widths[0] + widths[1]) / 2;
    }

    globalRoi =
        cv::Rect2f(rcenter.center.x - 0.5 * width, rcenter.center.y - 0.5 * width, width, width);
    if (status != Status::SUCCESS) {
        globalRoi = {0, 0, static_cast<float>(image_width_), static_cast<float>(image_height_)};
    }
    resetRoi(globalRoi, image_height_, image_width_);
}

void RuneDetector::setLocalRoi() {
    targetROIs.clear();

    // 临时保存所有 arrow 的上下 ROI
    std::vector<cv::Rect2f> rawUpROIs;
    std::vector<cv::Rect2f> rawDownROIs;

    for (auto arrow : arrows) {

        double distance{arrow.size.width * LOCAL_ROI_DISTANCE_RATIO};
        float width{LOCAL_ROI_WIDTH};

        float x = distance * std::cos(arrow.angle * CV_PI / 180.0f);
        float y = distance * std::sin(arrow.angle * CV_PI / 180.0f);

        cv::Point2f centerUp{arrow.center.x - globalRoi.x + x, arrow.center.y - globalRoi.y + y};
        cv::Point2f centerDown{arrow.center.x - x - globalRoi.x, arrow.center.y - globalRoi.y - y};

        cv::RotatedRect rectUp{centerUp, cv::Size(width, width), (float)arrow.angle};
        cv::RotatedRect rectDown{centerDown, cv::Size(width, width), (float)arrow.angle};

        std::array<std::array<cv::Point2f, 4>, 2> roiPoints;
        rectUp.points(roiPoints.at(0).begin());
        rectDown.points(roiPoints.at(1).begin());

        for (auto& pts : roiPoints) {
            for (auto& p : pts) {
                p.x = std::clamp(p.x, 0.f, globalRoi.width);
                p.y = std::clamp(p.y, 0.f, globalRoi.height);
            }
        }

        for (const auto& pts : roiPoints) {
            std::vector<cv::Point> ip;
            for (auto& p : pts)
                ip.emplace_back((int)p.x, (int)p.y);
            cv::fillConvexPoly(localMask, ip, cv::Scalar(255));
        }

        cv::Rect2f targetRoi(centerUp.x - width * 0.5f, centerUp.y - width * 0.5f, width, width);
        cv::Rect2f rcenterRoi(
            centerDown.x - width * 0.5f, centerDown.y - width * 0.5f, width, width);

        resetRoi(targetRoi, globalRoi.height, globalRoi.width);
        resetRoi(rcenterRoi, globalRoi.height, globalRoi.width);

        rawUpROIs.emplace_back(targetRoi);
        rawDownROIs.emplace_back(rcenterRoi);
    }

    int N = arrows.size();

    if (N == 1) {
        cv::Rect2f upROI   = rawUpROIs[0];
        cv::Rect2f downROI = rawDownROIs[0];

        cv::Rect2f downGlobal(
            downROI.x + globalRoi.x, downROI.y + globalRoi.y, downROI.width, downROI.height);

        if (inRect(rcenter.center, downGlobal)) {
            centerRoi  = downROI;
            targetROIs = {upROI};
        } else {
            centerRoi  = upROI;
            targetROIs = {downROI};
        }
        return;
    }

    if (N == 2) {
        cv::Rect2f A = rawUpROIs[0];
        cv::Rect2f B = rawDownROIs[0];
        cv::Rect2f C = rawUpROIs[1];
        cv::Rect2f D = rawDownROIs[1];

        double maxArea = 0.0;
        cv::Rect2f best1, best2;

        // Pair 1: A & C
        {
            cv::Rect2f inter = A & C;
            if (inter.area() > maxArea) {
                maxArea = inter.area();
                best1   = A;
                best2   = C;
            }
        }

        // Pair 2: A & D
        {
            cv::Rect2f inter = A & D;
            if (inter.area() > maxArea) {
                maxArea = inter.area();
                best1   = A;
                best2   = D;
            }
        }

        // Pair 3: B & C
        {
            cv::Rect2f inter = B & C;
            if (inter.area() > maxArea) {
                maxArea = inter.area();
                best1   = B;
                best2   = C;
            }
        }

        // Pair 3: B & D
        {
            cv::Rect2f inter = B & D;
            if (inter.area() > maxArea) {
                maxArea = inter.area();
                best1   = B;
                best2   = D;
            }
        }

        cv::Rect2f inter = best1 & best2;

        float w    = inter.width;
        float h    = inter.height;
        float side = std::max(w, h); // 正方形边长 = 最大边

        // 以 inter 的中心为正方形中心
        cv::Point2f center = (inter.tl() + inter.br()) * 0.5f;

        // 生成正方形 ROI
        centerRoi = cv::Rect2f(center.x - side / 2, center.y - side / 2, side, side);
        resetRoi(centerRoi, globalRoi.height, globalRoi.width);

        std::vector<cv::Rect2f> all = {A, B, C, D};

        for (auto& r : all) {
            if (r != best1 && r != best2) {
                targetROIs.push_back(r);
            }
        }
    }
}

bool sameArrow(const Light& l1, const Light& l2) {
    // 判断面积比
    double areaRatio{l1.area / l2.area};
    if ((areaRatio >= 1 / MAX_SAME_ARROW_AREA_RATIO && areaRatio <= MAX_SAME_ARROW_AREA_RATIO)
        == false) {
        return false;
    }
    // 判断距离
    double distance{cv::norm(l1.rotated.center - l2.rotated.center)};
    double maxDistance{1.2 * (l1.size.width + l2.size.width)};
    if (distance > maxDistance) {
        return false;
    }
    return true;
}

bool findArrow(Arrow& arrow, const std::vector<Light>& lights, cv::Rect2f& roi) {
    // 利用 cv::partition 匹配箭头
    std::vector<int> labels;
    cv::partition(lights, labels, sameArrow);
    // data 记录了标识号和其对应次数
    std::vector<std::pair<int, int>> data;
    for (auto label : labels) {
        // 对每个 label，从已记录的数据中寻找是否有这个条目，有则对应计数项
        // +1，否则新增一个条目
        auto iter =
            std::find_if(data.begin(), data.end(), [label](const std::pair<int, int>& unit) {
                return unit.first == label;
            });
        if (iter == data.end()) {
            data.emplace_back(label, 1);
        } else {
            iter->second += 1;
        }
    }
    if (data.empty() == true) {

        return false;
    }
    // 寻找出现次数最多的 label 和其对应的 num
    auto [maxLabel, maxNum]{*std::max_element(
        data.begin(), data.end(), [](const std::pair<int, int>& i, const std::pair<int, int>& j) {
            return i.second < j.second;
        })};
    // 判断 num 是否符合要求
    if ((maxNum >= MIN_ARROW_LIGHT_NUM && maxNum <= MAX_ARROW_LIGHT_NUM) == false) {
        return false;
    }
    // 再次遍历 labels，选取和 maxLabel 相同的 label，并存入一个向量
    std::vector<int> arrowIndices;
    for (size_t i = 0; i < labels.size(); ++i) {
        if (labels[i] == maxLabel) {
            arrowIndices.push_back(i);
        }
    }
    // 根据这个向量，将其对应的灯条轮廓点集中每个点存入箭头点的向量中
    std::vector<Light> arrowLights;
    for (auto index : arrowIndices) {
        arrowLights.push_back(lights.at(index));
    }
    // 设置这个箭头

    arrow.set(arrowLights, roi.tl());

    // 判断长宽比
    if ((arrow.ratio >= MIN_ARROW_ASPECT_RATIO && arrow.ratio <= MAX_ARROW_ASPECT_RATIO) == false) {
        return false;
    }
    // 判断面积
    if (arrow.area > MAX_ARROW_AREA) {
        return false;
    }
    return true;
}

void findArrowLights(const cv::Mat& binary, std::vector<Light>& lights, const cv::Rect2f& roi) {
    // 寻找轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
    for (const auto& contour : contours) {
        Light light(contour, roi, roi);

        if ((light.area >= MIN_ARROW_LIGHT_AREA && light.area <= MAX_ARROW_LIGHT_AREA) == false) {
            continue;
        }
        // 判断长宽比

        if (light.ratio > MAX_ARROW_LIGHT_ASPECT_RATIO) {
            continue;
        }
        // 符合要求，则存入
        lights.emplace_back(std::move(light));
    }
}

bool RuneDetector::detectAllArrows() {
    cv::Mat binImg = arrowImg.clone(); // 二值图
    arrows.clear();

    while (true) {
        std::vector<Light> lights;
        findArrowLights(binImg, lights, globalRoi);

        Arrow found;
        if (!findArrow(found, lights, globalRoi))
            break;                     // 找不到新箭头，退出循环

        arrows.push_back(found);

        // 用掩膜遮掉已经识别的箭头灯条
        cv::drawContours(
            binImg, std::vector<std::vector<cv::Point>>{found.contour}, -1, cv::Scalar(0),
            cv::FILLED);
    }

    if (arrows.empty()) {
        return false;
    }
    return true;
}

bool findCenterR(
    CenterR& center, const std::vector<Light>& lights, const Arrow& arrow, const Target& target) {
    // 设置中心 R 到靶中心的距离范围
    // const double distanceRTarget{arrow.size.width / ARROW_WIDTH * POWER_RUNE_RADIUS};
    // const double ratio = 0.5;
    // const double maxDistanceRTarget{distanceRTarget / ratio};
    // const double minDistanceRTarget{distanceRTarget * ratio};
    // // 设置中心 R 到箭头所在直线的最大距离
    // const double maxDistanceRArrow{arrow.size.width * 5};

    std::vector<Light> filteredLights;
    for (auto iter = lights.begin(); iter != lights.end(); ++iter) {

        if (!arrow.contour.empty()) {
            if (cv::pointPolygonTest(arrow.contour, iter->center, false) >= 0) {
                continue; // 在箭头内部，跳过
            }
        } else {
            // 如果箭头只有 RotatedRect，使用矩形判断
            if (arrow.rotated.boundingRect().contains(iter->center)) {
                continue;
            }
        }

        // double p2p{cv::norm(target.center - iter->center)};
        // double p2l{pointLineDistance(iter->center, target.center, arrow.center)};
        // if ((p2p >= minDistanceRTarget && p2p <= maxDistanceRTarget) == false) {
        //     continue;
        // }

        // if (p2l > maxDistanceRArrow) {
        //     continue;
        // }
        filteredLights.push_back(*iter);
    }
    if (filteredLights.empty()) {
        return false;
    }
    // 取所有符合要求的灯条中面积最大的为中心 R 灯条并设置中心 R

    Light rcenter{*std::max_element(
        filteredLights.begin(), filteredLights.end(),
        [](const Light& l1, const Light& l2) { return l1.area < l2.area; })};
    center.set(rcenter);
    return true;
}

static std::vector<cv::Point> normalizeContour(const std::vector<cv::Point>& cnt) {
    cv::Rect box = cv::boundingRect(cnt);
    std::vector<cv::Point> out;
    out.reserve(cnt.size());

    for (auto& p : cnt) {
        float nx = float(p.x - box.x) / float(box.width);
        float ny = float(p.y - box.y) / float(box.height);
        out.emplace_back(int(nx * 1000), int(ny * 1000));
    }
    return out;
}

bool findCenterLights(
    const cv::Mat& image, std::vector<Light>& lights, const cv::Rect2f& globalRoi,
    const cv::Rect2f& localRoi) {
    lights.clear();

    cv::Mat R_mat;
    cv::FileStorage fs("/home/developer/ws/assets/r_template0.yml", cv::FileStorage::READ);
    fs["R_contour"] >> R_mat;
    fs.release();

    if (R_mat.empty())
        return false;

    // 转为 vector<Point>
    std::vector<cv::Point> R_cnt;
    R_cnt.reserve(R_mat.rows);
    for (int i = 0; i < R_mat.rows; i++) {
        cv::Vec<int, 2> v = R_mat.at<cv::Vec<int, 2>>(i, 0);
        R_cnt.emplace_back(v[0], v[1]);
    }

    auto R_norm = normalizeContour(R_cnt);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(image, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    double best_score = 1e9;
    std::vector<cv::Point> best_cnt;

    for (auto& cnt : contours) {
        if (cv::contourArea(cnt) < 80)
            continue;
        auto cnt_norm = normalizeContour(cnt);
        double score  = cv::matchShapes(cnt_norm, R_norm, cv::CONTOURS_MATCH_I1, 0);
        if (score < best_score) {
            best_score = score;
            best_cnt   = cnt;
        }
    }

    if (!best_cnt.empty() && best_score < 0.3) {
        Light light(best_cnt, globalRoi, localRoi);
        // 判断长宽比
        if (light.ratio > MAX_CENTER_ASPECT_RATIO) {
            return false;
        }
        // 如果全部符合，则存入向量中
        lights.emplace_back(std::move(light));
    } else {
        return false;
    }
    if (lights.empty()) {
        return false;
    }
    return true;
}

/*寻找R中心*/
bool RuneDetector::detectCenterR() {
    cv::Mat imageCenter = (rCenterImg & localMask)(centerRoi);
    // 寻找中心灯条，可能是多个
    std::vector<Light> lights;
    if (findCenterLights(imageCenter, lights, globalRoi, centerRoi) == false) {

        return false;
    }

    // 从灯条中寻找中心 R
    if (findCenterR(rcenter, lights, arrows[0], targets[0]) == false) {
        return false;
    }

    return true;
}

/*通过面积，轮廓面积，长宽比，判断目标是否符合*/
bool findTargetLights(
    const cv::Mat& image, std::vector<Light>& lights, const cv::Rect2f& globalRoi,
    const cv::Rect2f& localRoi) {
    lights.clear();
    // 寻找轮廓
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(image, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty()) {
        return false;
    }
    for (size_t i = 0; i < contours.size(); i++) {

        int child_idx  = hierarchy[i][2];
        int parent_idx = hierarchy[i][3];
        if (child_idx == -1 && parent_idx == -1) {
            continue;
        }

        Light light(contours[i], globalRoi, localRoi);

        if (light.ratio < MIN_TARGET_LIGHT_ASPECT_RATIO
            || light.ratio > MAX_TARGET_LIGHT_ASPECT_RATIO) {
            continue;
        }

        if (light.area < MIN_TARGET_LIGHT_AREA || light.area > MAX_TARGET_LIGHT_AREA) {
            continue;
        }

        if (light.contourArea < MIN_TARGET_LIGHT_CONTOUR_AREA
            || light.contourArea > MAX_TARGET_LIGHT_CONTOUR_AREA) {
            continue;
        }

        lights.emplace_back(std::move(light));
    }

    if (lights.empty()) {
        return false;
    }

    return true;
}

bool findTarget(Target& target, const std::vector<Light>& frames) {

    std::vector<Light> targets;

    for (auto frame : frames) {
        if ((frame.roundness >= MIN_ROUNDNESS && frame.roundness <= MAX_ROUNDNESS) == false) {
            continue;
        }

        targets.emplace_back(frame);
    }
    if (targets.empty()) {

        return false;
    }
    Light light{
        *std::max_element(targets.begin(), targets.end(), [](const Light& l1, const Light& l2) {
            return l1.roundness < l2.roundness;
        })};

    target.set(light);

    return true;
}

bool RuneDetector::detectAllTargets() {
    std::vector<Light> lights;
    targets.clear();
    bool reverse   = false;
    cv::Mat backup = (targetImg & localMask)(centerRoi);
    if (targetROIs.size() == 1) {
        cv::Rect2f targetRoi = targetROIs[0];
        cv::Mat detect       = (targetImg & localMask)(targetRoi);

    RESTART:
        if (!findTargetLights(detect, lights, globalRoi, targetRoi)) {
            if (!reverse) {
                std::swap(detect, backup);
                std::swap(targetRoi, centerRoi);
                reverse = true;
                goto RESTART;
            } else {
                return false;
            }
        }

        Target t;
        if (!findTarget(t, lights)) {
            if (!reverse) {
                std::swap(detect, backup);
                std::swap(targetRoi, centerRoi);
                reverse = true;
                goto RESTART;
            } else {
                return false;
            }
        }

        targets.push_back(t);
    } else {
        for (auto targetRoi : targetROIs) {
            lights.clear();
            cv::Mat detect = (targetImg & localMask)(targetRoi);

            if (findTargetLights(detect, lights, globalRoi, targetRoi) == false) {
                continue;
            }
            Target t;
            if (findTarget(t, lights) == false) {
                continue;
            }
            targets.emplace_back(t);
        }
    }
    if (targets.empty()) {
        return false;
    }
    return true;
}

struct Points {
    cv::Point2f center;
    std::vector<cv::Point2f> corners;
};

bool sameTarget(const std::vector<cv::Point>& contour1, const std::vector<cv::Point>& contour2) {
    // 判断面积比
    double areaRatio{cv::contourArea(contour1) / cv::contourArea(contour2)};
    if ((areaRatio >= 0.75 && areaRatio <= 1.25) == false) {
        return false;
    }
    cv::RotatedRect rotated1 = cv::minAreaRect(contour1);
    cv::RotatedRect rotated2 = cv::minAreaRect(contour2);
    // 判断距离
    double distance{cv::norm(rotated1.center - rotated2.center)};
    double maxDistance{1.5 * (rotated1.size.width + rotated2.size.width)};
    if (distance > maxDistance) {
        return false;
    }

    // 判断轮廓旋转矩形的长宽
    if (rotated1.size.height < rotated1.size.width) {
        cv::swap(rotated1.size.height, rotated1.size.width);
    }
    if (rotated2.size.height < rotated2.size.width) {
        cv::swap(rotated2.size.height, rotated2.size.width);
    }

    if (rotated1.size.width - rotated2.size.width > 5
        || rotated1.size.height - rotated2.size.height > 5) {
        return false;
    }
    return true;
}


void addReferRuneCenter(const CenterR& rc, Points& target) {

    if (target.corners.size() != 4)
        return;

    cv::Point2f down_vec = rc.center - target.center;
    float norm           = std::sqrt(down_vec.x * down_vec.x + down_vec.y * down_vec.y);
    if (norm < 1e-6f)
        return;
    
    float angle_ref = std::atan2(down_vec.y, down_vec.x);

    // 获取4个点在旋转后的角度
    struct Node {
        float ang;
        cv::Point2f p;
    };
    std::vector<Node> arr;
    arr.reserve(4);

    for (auto& p : target.corners) {
        cv::Point2f v = p - target.center;

        // 旋转坐标，使 down_vec 对齐 angle=0
        float ang = std::atan2(v.y, v.x) - angle_ref;

        // 归一化到 (-π, π]
        while (ang <= -CV_PI)
            ang += 2 * CV_PI;
        while (ang > CV_PI)
            ang -= 2 * CV_PI;

        arr.push_back({ang, p});
    }

    // 按角度排序（从 -π 到 π）
    std::sort(arr.begin(), arr.end(), [](const Node& a, const Node& b) { return a.ang < b.ang; });

    // 准备象限变量并标记
    cv::Point2f lu(0, 0), ru(0, 0), rd(0, 0), ld(0, 0);
    bool has_lu = false, has_ru = false, has_rd = false, has_ld = false;

    for (const auto& n : arr) {
        float a = n.ang;

        if (a > CV_PI / 2 && a <= CV_PI) {
            lu     = n.p;
            has_lu = true;
        } else if (a > 0 && a <= CV_PI / 2) {
            ru     = n.p;
            has_ru = true;
        } else if (a > -CV_PI / 2 && a <= 0) {
            rd     = n.p;
            has_rd = true;
        } else {                           // a > -CV_PI && a <= -CV_PI/2
            ld     = n.p;
            has_ld = true;
        }
    }

    std::array<cv::Point2f, 4> ordered;

    if (has_lu && has_ru && has_rd && has_ld) {
        ordered[0] = lu;
        ordered[1] = ru;
        ordered[2] = rd;
        ordered[3] = ld;
        target.corners.assign(ordered.begin(), ordered.end());
        return;
    }

    float angle     = 3.0f * CV_PI / 4.0f; // 135°
    int best_idx    = 0;
    float best_diff = std::numeric_limits<float>::max();
    for (int i = 0; i < (int)arr.size(); ++i) {
        float d = std::fabs(
            angles::shortest_angular_distance(
                angle, arr[i].ang)); 
        if (d < best_diff) {
            best_diff = d;
            best_idx  = i;
        }
    }

    for (int i = 0; i < 4; ++i) {
        int idx    = (best_idx + i) % 4;
        ordered[i] = arr[idx].p;
    }

    target.corners.assign(ordered.begin(), ordered.end());
}


inline Target markRuneTarget(
    std::vector<cv::Point> arrow, CenterR rc, Target target,
    const std::vector<std::vector<cv::Point>>& contours, const std::vector<cv::Vec4i>& hierarchy) {

    Target result;
    if (hierarchy.empty()) {
        std::cerr << "hierarch empty" << std::endl;
        return result;
    }

    std::vector<std::vector<cv::Point>> c;
    for (size_t k = 0; k < contours.size(); k++) {
        if (hierarchy[k][3] == -1 || (hierarchy[k][0] == -1 && hierarchy[k][1] == -1)) {
            continue;
        }
        cv::RotatedRect rotate = cv::minAreaRect(contours[k]);
        cv::Point2f ves[4];
        bool including = false;
        rotate.points(ves);
        for (int i = 0; i < 4; i++) {
            int j = cv::pointPolygonTest(arrow, ves[i], false);
            if (j > 0) {
                including = true;
                break;
            }
        }
        if (!including) {
            c.emplace_back(contours[k]);
        }
    }

    std::vector<int> labels;
    cv::partition(c, labels, sameTarget);

    // data 记录了标识号和其对应次数
    std::vector<std::pair<int, int>> datas;
    for (auto label : labels) {
        // 对每个 label，从已记录的数据中寻找是否有这个条目，有则对应计数项
        // +1，否则新增一个条目
        auto iter =
            std::find_if(datas.begin(), datas.end(), [label](const std::pair<int, int>& unit) {
                return unit.first == label;
            });
        if (iter == datas.end()) {
            datas.emplace_back(label, 1);
        } else {
            iter->second += 1;
        }
    }
    if (datas.empty() == true) {
        return result;
    }

    std::vector<std::vector<cv::Point>> res;
    for (auto data : datas) {
        int num = data.second;
        if ((num > 3 && num < 7) == false) {
            continue;
        }

        for (size_t i = 0; i < c.size(); i++)
            if (labels[i] == data.first) {
                res.emplace_back(c[i]);
            }
    }

    Points points;

    for (auto cnt : res) {

        cv::Moments m = cv::moments(cnt);
        if (m.m00 == 0) {
            std::cerr << "m.m00 == 0" << std::endl;
            continue;
        }

        // 质心
        cv::Point2f center(m.m10 / m.m00, m.m01 / m.m00);
        points.corners.emplace_back(center);
    }
    points.center = target.center;

    addReferRuneCenter(rc, points);
 
    result.keypnt.lu = points.corners[0];
    result.keypnt.ru = points.corners[1];
    result.keypnt.rd = points.corners[2];
    result.keypnt.ld = points.corners[3];

    return result;
}

void RuneDetector::setKeyPoints() {
    for (size_t i = 0; i < targets.size(); i++) {

        cv::Mat detect = (targetImg & localMask)(targetROIs[i]);
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(detect, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);

        std::vector<cv::Point> arrow_contour;
        for (auto cnt_pt : arrows[i].contour) {
            cv::Point pt = cnt_pt - cv::Point(globalRoi.tl()) - cv::Point(targetROIs[i].tl());
            arrow_contour.emplace_back(pt);
        }

        Target rune_target =
            markRuneTarget(arrow_contour, rcenter, targets[i], contours, hierarchy);

        targets[i].keypnt.lu = rune_target.keypnt.lu + globalRoi.tl() + targetROIs[i].tl();
        targets[i].keypnt.ru = rune_target.keypnt.ru + globalRoi.tl() + targetROIs[i].tl();
        targets[i].keypnt.rd = rune_target.keypnt.rd + globalRoi.tl() + targetROIs[i].tl();
        targets[i].keypnt.ld = rune_target.keypnt.ld + globalRoi.tl() + targetROIs[i].tl();
    }
}


bool RuneDetector::detect(const cv::Mat& frame, int image_width, int image_height) {
    image_width_  = image_width;
    image_height_ = image_height;
    localMask     = cv::Mat::zeros(image_height_, image_width_, CV_8U);
    globalRoi =
        cv::Rect2f(0, 0, static_cast<float>(image_width_), static_cast<float>(image_height_));

    bool reverse = false;
    processPictures(frame);

    if (detectAllArrows() == false) {
        arrows.clear();
        targets.clear();
        rcenter.center = cv::Point2f(0, 0);
        status         = Status::ARROW_FAILURE;
        goto FAIL;
    }

    setLocalRoi();

RESTART:

    if (detectAllTargets() == false) {
        targets.clear();
        status = Status::ARMOR_FAILURE;
        goto FAIL;
    }

    if (targetROIs.size() == 1) {
        if (detectCenterR() == false) {
            status         = Status::CENTER_FAILURE;
            rcenter.center = cv::Point2f(0, 0);
            if (reverse == false) {
                std::swap(centerRoi, targetROIs[0]);
                reverse = true;
                goto RESTART;
            } else {
                goto FAIL;
            }
        }
    } else {
        if (detectCenterR() == false) {
            status         = Status::CENTER_FAILURE;
            rcenter.center = cv::Point2f(0, 0);
            goto FAIL;
        }
    }

    status = Status::SUCCESS;
    // setGlobalRoi();

    return true;
FAIL:

    // 如果检测失败，则将全局 roi 设为和原图片一样大小
    // globalRoi     = {0, 0, static_cast<float>(frame.cols), static_cast<float>(frame.rows)};
    return false;
}

RuneDetector::RuneDetector(
    EnemyColor detect_color, int arrow_threshold, int target_threshold, int rcenter_threshold)
    : detect_color_(detect_color)
    , arrow_threshold_(arrow_threshold)
    , target_threshold_(target_threshold)
    , rcenter_threshold_(rcenter_threshold) {}

RuneDetector::RuneDetector() {}

Eigen::Matrix4d RuneDetector::solve(
    const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs, const rclcpp::Time& stamp,
    tf2_ros::Buffer& tf2_buffer) {
    if (targets.empty()) {
        return Eigen::Matrix4d::Identity();
    }
    std::vector<cv::Point3f> object_points;
    std::vector<cv::Point2f> image_points;

    object_points.clear();
    image_points.clear();

    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    cv::Mat rvec, tvec;

    if (targets.size() == 1) {

        object_points = {
            {0.0,                           0.0,                                               0.0},
            {0.0,                           0.0,                       (float)(-POWER_RUNE_RADIUS)},
            {0.0,                           0.0, (float)(-POWER_RUNE_RADIUS + POWER_TARGET_RADIUS)},
            {0.0,    (float)POWER_TARGET_RADIUS,                       (float)(-POWER_RUNE_RADIUS)},
            {0.0,                           0.0, (float)(-POWER_RUNE_RADIUS - POWER_TARGET_RADIUS)},
            {0.0, (float)(-POWER_TARGET_RADIUS),                       (float)(-POWER_RUNE_RADIUS)}
        };

        image_points.emplace_back(rcenter.center);
        image_points.emplace_back(targets[0].center);
        // image_points.emplace_back(targets[0].keypnt.down);  // 下
        // image_points.emplace_back(targets[0].keypnt.left);  // 左
        // image_points.emplace_back(targets[0].keypnt.up);    // 上
        // image_points.emplace_back(targets[0].keypnt.right); // 右

        // 校验数量匹配
        if (object_points.size() != image_points.size()) {
            std::cerr << "3D point number not equals 2D point number" << std::endl;
            return Eigen::Matrix4d::Identity();
        }

        // 2. 点数数量校验（至少4个点，满足solvePnP要求）
        if (object_points.size() < 4) {
            std::cerr << "pnp point number less 4" << std::endl;
            return Eigen::Matrix4d::Identity();
        }

        bool success = cv::solvePnP(
            object_points, image_points, cameraMatrix, distCoeffs, rvec, tvec, false,
            cv::SOLVEPNP_SQPNP);
        if (!success) {
            std::cerr << "R点PnP解算失败" << std::endl;
            return Eigen::Matrix4d::Identity();
        }

    } else if (targets.size() == 2) {
        if (targets[0].center.y < targets[1].center.y) {
            std::swap(targets[0], targets[1]);
            std::swap(arrows[0], arrows[1]);
        }

        double distancetTargetToTarget = cv::norm(targets[0].center - targets[1].center);
        double distanceTargetToRcenter = cv::norm(targets[0].center - rcenter.center);
        double sin36                   = std::sin(36.0 * CV_PI / 180.0);
        double sin72                   = std::sin(72.0 * CV_PI / 180.0);
        double cos72                   = std::cos(72.0 * CV_PI / 180.0);
        double sin54                   = std::sin(54.0 * CV_PI / 180.0);
        double cos54                   = std::cos(54.0 * CV_PI / 180.0);

        double theo36 = 2.0 * distanceTargetToRcenter * sin36;
        double theo72 = 2.0 * distanceTargetToRcenter * sin72;
        double eps    = 50.0;

        image_points.emplace_back(rcenter.center);
        image_points.emplace_back(targets[0].center);
        // image_points.emplace_back(targets[0].keypnt.down);  // 下
        // image_points.emplace_back(targets[0].keypnt.left);  // 左
        // image_points.emplace_back(targets[0].keypnt.up);    // 上
        // image_points.emplace_back(targets[0].keypnt.right); // 右
        // image_points.emplace_back(targets[1].center);
        // image_points.emplace_back(targets[1].keypnt.down);  // 下
        // image_points.emplace_back(targets[1].keypnt.left);  // 左
        // image_points.emplace_back(targets[1].keypnt.up);    // 上
        // image_points.emplace_back(targets[1].keypnt.right); // 右

        if (std::abs(distancetTargetToTarget - theo36) < eps) {
            object_points = {
                {0.0,                                                              0.0,0.0                                                                                       },
                {0.0,                                                              0.0,                       (float)(-POWER_RUNE_RADIUS)},
                {0.0,                                                              0.0, (float)(-POWER_RUNE_RADIUS + POWER_TARGET_RADIUS)},
                {0.0,                                       (float)POWER_TARGET_RADIUS,                       (float)(-POWER_RUNE_RADIUS)},
                {0.0,                                                              0.0, (float)(-POWER_RUNE_RADIUS - POWER_TARGET_RADIUS)},
                {0.0,                                    (float)(-POWER_TARGET_RADIUS),                       (float)(-POWER_RUNE_RADIUS)},
                {0.0,                               (float)(POWER_RUNE_RADIUS * sin72),             (float)(-(POWER_RUNE_RADIUS * cos72))},
                {0.0,       (float)((POWER_RUNE_RADIUS - POWER_TARGET_RADIUS) * sin72),
                 (float)(-(POWER_RUNE_RADIUS - POWER_TARGET_RADIUS) * cos72)                                                             },
                {0.0, (float)(POWER_RUNE_RADIUS * sin72 + POWER_TARGET_RADIUS * cos72),
                 (float)(-(POWER_RUNE_RADIUS * cos72 - POWER_TARGET_RADIUS * sin72))                                                     },
                {0.0,       (float)((POWER_RUNE_RADIUS + POWER_TARGET_RADIUS) * sin72),
                 (float)(-((POWER_RUNE_RADIUS + POWER_TARGET_RADIUS) * cos72))                                                           },
                {0.0, (float)(POWER_RUNE_RADIUS * sin72 - POWER_TARGET_RADIUS * cos72),
                 (float)(-(POWER_RUNE_RADIUS * cos72 + POWER_TARGET_RADIUS * sin72))                                                     }
            };
            // std::cerr << "72度，两个靶相邻" << std::endl;

            // 校验数量匹配
            if (object_points.size() != image_points.size()) {
                std::cerr << "3D point number not equals 2D point number" << std::endl;
                return Eigen::Matrix4d::Identity();
            }

            // 2. 点数数量校验（至少4个点，满足solvePnP要求）
            if (object_points.size() < 4) {
                std::cerr << "pnp point number less 4" << std::endl;
                return Eigen::Matrix4d::Identity();
            }

            bool success = cv::solvePnPRansac(
                object_points, image_points, cameraMatrix, distCoeffs, rvec, tvec);
            if (!success) {
                std::cerr << "R点PnP解算失败" << std::endl;
                return Eigen::Matrix4d::Identity();
            }
        } else if (std::abs(distancetTargetToTarget - theo72) < eps) {
            object_points = {
                {0.0,                                                              0.0,0.0                                                                                       },
                {0.0,                                                              0.0,                       (float)(-POWER_RUNE_RADIUS)},
                {0.0,                                                              0.0, (float)(-POWER_RUNE_RADIUS + POWER_TARGET_RADIUS)},
                {0.0,                                       (float)POWER_TARGET_RADIUS,                       (float)(-POWER_RUNE_RADIUS)},
                {0.0,                                                              0.0, (float)(-POWER_RUNE_RADIUS - POWER_TARGET_RADIUS)},
                {0.0,                                    (float)(-POWER_TARGET_RADIUS),                       (float)(-POWER_RUNE_RADIUS)},
                {0.0,                               (float)(POWER_RUNE_RADIUS * cos54),                (float)(POWER_RUNE_RADIUS * sin54)},
                {0.0,       (float)((POWER_RUNE_RADIUS - POWER_TARGET_RADIUS) * cos54),
                 (float)((POWER_RUNE_RADIUS - POWER_TARGET_RADIUS) * sin54)                                                              },
                {0.0, (float)(POWER_RUNE_RADIUS * cos54 - POWER_TARGET_RADIUS * sin54),
                 (float)(POWER_RUNE_RADIUS * sin54 + POWER_TARGET_RADIUS * cos54)                                                        },
                {0.0,       (float)((POWER_RUNE_RADIUS + POWER_TARGET_RADIUS) * cos54),
                 (float)((POWER_RUNE_RADIUS + POWER_TARGET_RADIUS) * sin54)                                                              },
                {0.0, (float)(POWER_RUNE_RADIUS * cos54 + POWER_TARGET_RADIUS * sin54),
                 (float)(POWER_RUNE_RADIUS * sin54 - POWER_TARGET_RADIUS * cos54)                                                        }
            };
            // std::cerr << "144度，两个靶中隔一个靶" << std::endl;

            // 校验数量匹配
            if (object_points.size() != image_points.size()) {
                std::cerr << "3D point number not equals 2D point number" << std::endl;
                return Eigen::Matrix4d::Identity();
            }

            // 2. 点数数量校验（至少4个点，满足solvePnP要求）
            if (object_points.size() < 4) {
                std::cerr << "pnp point number less 4" << std::endl;
                return Eigen::Matrix4d::Identity();
            }

            bool success = cv::solvePnP(
                object_points, image_points, cameraMatrix, distCoeffs, rvec, tvec, false,
                cv::SOLVEPNP_IPPE);
            if (!success) {
                std::cerr << "R点PnP解算失败" << std::endl;
                return Eigen::Matrix4d::Identity();
            }
        }
    }

    cv::Mat rmat;
    try {
        cv::Rodrigues(rvec, rmat);
    } catch (const cv::Exception& e) {
        std::cerr << "R点旋转矩阵转换失败: " << e.what() << std::endl;
        return Eigen::Matrix4d::Identity();
    }

    Eigen::Matrix3d rot;
    try {
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                rot(i, j) = rmat.at<double>(i, j);
            }
        }
    } catch (const cv::Exception& e) {
        std::cerr << "R点数据转换失败: " << e.what() << std::endl;
        return Eigen::Matrix4d::Identity();
    }
    Eigen::Quaterniond quat(rot);

    geometry_msgs::msg::PoseStamped ps_cam, ps_odom;
    ps_cam.header.frame_id = "camera_optical_frame";
    ps_cam.header.stamp    = stamp;

    ps_cam.pose.position.x    = tvec.at<double>(0);
    ps_cam.pose.position.y    = tvec.at<double>(1);
    ps_cam.pose.position.z    = tvec.at<double>(2);
    ps_cam.pose.orientation.x = quat.x();
    ps_cam.pose.orientation.y = quat.y();
    ps_cam.pose.orientation.z = quat.z();
    ps_cam.pose.orientation.w = quat.w();

    try {
        ps_odom = tf2_buffer.transform(ps_cam, "odom");
    } catch (const tf2::TransformException& ex) {
        std::cerr << "R点TF转换失败: " << ex.what() << std::endl;
        return Eigen::Matrix4d::Identity();
    }

    pose(0, 3) = ps_odom.pose.position.x;
    pose(1, 3) = ps_odom.pose.position.y;
    pose(2, 3) = ps_odom.pose.position.z;
    Eigen::Quaterniond quat_odom(
        ps_odom.pose.orientation.w, ps_odom.pose.orientation.x, ps_odom.pose.orientation.y,
        ps_odom.pose.orientation.z);
    pose.block<3, 3>(0, 0) = quat_odom.toRotationMatrix();

    return pose;
}
} // namespace fyt::rune