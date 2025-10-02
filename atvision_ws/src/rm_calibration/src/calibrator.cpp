#include "rm_calibration/calibrator.hpp"

namespace rm_calibration {

Calibrator::Calibrator() {
    K_ = cv::Mat::eye(3, 3, CV_64F);
    D_ = cv::Mat::zeros(1, 5, CV_64F);
}

void Calibrator::setPattern(int cols, int rows, double spacing_mm, PatternType type) {
    cols_       = cols;
    rows_       = rows;
    spacing_mm_ = spacing_mm;
    pattern_    = type;
}

void Calibrator::setCameraIntrinsics(const cv::Mat& K, const cv::Mat& D) {
    K_ = K;
    D_ = D;
}

void Calibrator::setRGimbal2ImuBody(const Eigen::Matrix3d& R_g2ib) { R_gimbal2imubody_ = R_g2ib; }

void Calibrator::setOutputYamlPath(const std::string& path) { yaml_out_ = path; }

void Calibrator::pushFrame(const cv::Mat& img_bgr, const Eigen::Quaterniond& imu_q) {
    std::lock_guard<std::mutex> lk(mtx_);
    Frame s;
    s.image = img_bgr.clone();
    s.imu_q = imu_q;
    buf_.emplace_back(std::move(s));
}

size_t Calibrator::sampleCount() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return buf_.size();
}

bool Calibrator::detectPattern(const cv::Mat& img, std::vector<cv::Point2f>& img_points) const {
    cv::Mat gray;
    if (img.channels() == 3)
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    else
        gray = img;

    const cv::Size pat(cols_, rows_);
    bool found = false;
    if (pattern_ == PatternType::SymmetricCircles) {
        found = cv::findCirclesGrid(gray, pat, img_points, cv::CALIB_CB_SYMMETRIC_GRID);
    } else {
        found = cv::findChessboardCorners(gray, pat, img_points);
        if (found) {
            cv::cornerSubPix(
                gray, img_points, cv::Size(5, 5), cv::Size(-1, -1),
                cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 50, 1e-3));
        }
    }
    return found;
}

void Calibrator::buildPlanarObjectPoints(std::vector<cv::Point3f>& obj_pts) const {
    obj_pts.clear();
    obj_pts.reserve(cols_ * rows_);
    for (int i = 0; i < rows_; ++i)
        for (int j = 0; j < cols_; ++j)
            obj_pts.emplace_back(float(j * spacing_mm_), float(i * spacing_mm_), 0.0f);
}

cv::Mat Calibrator::eigenRToCv(const Eigen::Matrix3d& R) {
    cv::Mat Rc(3, 3, CV_64F);
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            Rc.at<double>(r, c) = R(r, c);
    return Rc;
}

double Calibrator::calibrateCameraFromBuffer() {
    std::vector<std::vector<cv::Point3f>> obj_points;
    std::vector<std::vector<cv::Point2f>> img_points;
    cv::Size img_size;

    {
        std::lock_guard<std::mutex> lk(mtx_);
        if (buf_.empty())
            return -1.0;

        std::vector<cv::Point3f> obj;
        buildPlanarObjectPoints(obj);

        for (const auto& s : buf_) {
            std::vector<cv::Point2f> pts2d;
            if (!detectPattern(s.image, pts2d))
                continue;

            img_points.emplace_back(std::move(pts2d));
            obj_points.emplace_back(obj);
            if (img_size.empty())
                img_size = s.image.size();
        }
    }

    if (img_points.size() < 6)
        return -1.0;

    std::vector<cv::Mat> rvecs, tvecs;
    auto criteria =
        cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 100, DBL_EPSILON);

    cv::Mat K, D;
    double err = cv::calibrateCamera(
        obj_points, img_points, img_size, K, D, rvecs, tvecs, cv::CALIB_FIX_K3, criteria);
    setCameraIntrinsics(K, D);
    return err / (cols_ * rows_); // 每点平均（近似）
}

size_t Calibrator::buildPnPAndRobotMotions(
    std::vector<cv::Mat>& rvecs, std::vector<cv::Mat>& tvecs,
    std::vector<cv::Mat>& R_gimbal2world_list, std::vector<cv::Mat>& t_gimbal2world_list) const {
    rvecs.clear();
    tvecs.clear();
    R_gimbal2world_list.clear();
    t_gimbal2world_list.clear();

    std::vector<cv::Point3f> obj;
    buildPlanarObjectPoints(obj);

    std::lock_guard<std::mutex> lk(mtx_);
    if (buf_.empty())
        return 0;

    const cv::Size pattern(cols_, rows_);

    for (const auto& s : buf_) {
        // 1) 角点
        std::vector<cv::Point2f> pts2d;
        if (!detectPattern(s.image, pts2d)) {
            continue;
        }

        // 2) 质量门控（不通过则跳过该帧）
        const auto qr = qualityGate(s.image, pts2d, pattern, qp_);
        if (!qr.ok) {
            continue;
        }

        // 3) PnP (board->camera)
        cv::Mat rvec, tvec;
        cv::solvePnP(obj, pts2d, K_, D_, rvec, tvec, false, cv::SOLVEPNP_IPPE);

        // 4) 机器人姿态：gimbal->world
        Eigen::Matrix3d R_imubody2imuabs = s.imu_q.normalized().toRotationMatrix();
        Eigen::Matrix3d R_g2w =
            R_gimbal2imubody_.transpose() * R_imubody2imuabs * R_gimbal2imubody_;
        cv::Mat R_g2w_cv = eigenRToCv(R_g2w);
        cv::Mat t_g2w_cv = (cv::Mat_<double>(3, 1) << 0, 0, 0); // 仅用姿态，平移设 0

        // 5) 记录
        rvecs.emplace_back(rvec);
        tvecs.emplace_back(tvec);
        R_gimbal2world_list.emplace_back(R_g2w_cv);
        t_gimbal2world_list.emplace_back(t_g2w_cv);
    }
    return rvecs.size();
}

bool Calibrator::calibrateHandEye(cv::Mat& R_camera2gimbal, cv::Mat& t_camera2gimbal_m) {
    if (!has_intrinsics_) {
        if (calibrateCameraFromBuffer() < 0)
            return false;
    }

    std::vector<cv::Mat> rvecs, tvecs, R_g2w, t_g2w;
    size_t n = buildPnPAndRobotMotions(rvecs, tvecs, R_g2w, t_g2w);
    if (n < 6)
        return false;

    cv::calibrateHandEye(R_g2w, t_g2w, rvecs, tvecs, R_camera2gimbal, t_camera2gimbal_m);

    // mm -> m
    t_camera2gimbal_m /= 1000.0;
    return true;
}

bool Calibrator::writeYaml(
    const cv::Mat& R_camera2gimbal,
    const cv::Mat& t_camera2gimbal_m,
    const std::optional<cv::Mat>& R_world2board,
    const std::optional<cv::Mat>& t_world2board_m) const
{
    try {
        cv::FileStorage fs(yaml_out_, cv::FileStorage::WRITE | cv::FileStorage::FORMAT_YAML);
        if (!fs.isOpened()) return false;

        fs << "model" << "handeye";
        fs << "R_camera2gimbal" << R_camera2gimbal;   // 3x3
        fs << "t_camera2gimbal_m" << t_camera2gimbal_m; // 3x1

        if (has_intrinsics_) {
            fs << "K" << K_;
            fs << "D" << D_;
        }

        if (R_world2board && t_world2board_m) {
            fs << "R_world2board" << *R_world2board;
            fs << "t_world2board_m" << *t_world2board_m;
        }

        fs.release();
        return true;
    } catch (...) {
        return false;
    }
}
} // namespace rm_calibration
