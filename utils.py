# -*- coding: utf-8 -*-
# @Time    : 2021/7/24 下午1:51
# @Author  : red0orange
# @File    : utils.py
# @Content :
import numpy as np


def get_homo_matrix(R, T):
    # 得到齐次矩阵
    assert R.shape == (3, 3)
    assert T.shape == (3,) or T.shape == (3, 1)
    if T.shape == (3,):
        return_T = T[:, None]
    else:
        return_T = T
    return_R = np.concatenate([R, np.array([[0, 0, 0]])], axis=0)
    return_T = np.concatenate([return_T, np.array([[1]])], axis=0)
    HomoMatrix = np.concatenate([return_R, return_T], axis=1)
    return HomoMatrix


def get_RT_from_homo_matrix(homo_matrix):
    assert homo_matrix.shape == (4, 4)
    R = homo_matrix[:3, :3]
    T = homo_matrix[:3, -1]
    return R, T


def get_zero_translate_homo(R):
    zero_translate_matrix = np.array([0.0, 0.0, 0.0])
    return get_homo_matrix(R, zero_translate_matrix)

