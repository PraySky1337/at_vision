import numpy as np

# 矩阵相乘函数
def matmul(A, B):
    if A.ndim == 1:
        A = A[:,None]
    if B.ndim == 1:
        B = B[None,:]
    return A @ B