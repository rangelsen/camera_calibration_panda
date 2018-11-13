import numpy as np
import numpy.matlib as npm
import csv
import pyquaternion as pq
import math
import sys

################################################################################
def averageQuaternions(Q):
    # Number of quaternions to average
    M = Q.shape[0]
    A = npm.zeros(shape=(4,4))

    for i in range(0,M):
        q = Q[i,:]
        # multiply q with its transposed version q' and add A
        A = np.outer(q,q) + A

    # scale
    A = (1.0 / M)*A

    # compute eigenvalues and -vectors
    eigenValues, eigenVectors = np.linalg.eig(A)

    # Sort by largest eigenvalue
    eigenVectors = eigenVectors[:,eigenValues.argsort()[::-1]]

    # return the real part of the largest eigenvector (has only real part)
    return np.real(eigenVectors[:,0].A1)

################################################################################
def weightedAverageQuaternions(Q, w):
	# Number of quaternions to average
	M = Q.shape[0]
	A = npm.zeros(shape=(4,4))
	weightSum = 0

	for i in range(0,M):
		q = Q[i,:]
		A = w[i] * np.outer(q,q) + A
		weightSum += w[i]

	# scale
	A = (1.0/weightSum) * A

	# compute eigenvalues and -vectors
	eigenValues, eigenVectors = np.linalg.eig(A)

	# Sort by largest eigenvalue
	eigenVectors = eigenVectors[:,eigenValues.argsort()[::-1]]

	# return the real part of the largest eigenvector (has only real part)
	return np.real(eigenVectors[:,0].A1)

################################################################################
def parseRow(row_str):
	
	tf = np.empty([4, 4], dtype=float)

	for i in range(0, 4):
		for j in range(0, 4):
			tf[i][j] = float(row_str[i * 4 + j + 1])
			# print("tf[", i, "][", j, "] = ", tf[i][j])

	return tf
			
################################################################################
def normalize(vec):
	return vec / np.linalg.norm(vec)

################################################################################
def normalizeRotation(rot):
	rot_out = np.empty([3, 3], dtype=float)
	rot_out[:, 0] = normalize(np.cross(rot[:, 1], rot[:, 2]))
	rot_out[:, 1] = normalize(np.cross(rot[:, 2], rot_out[:, 0]))
	rot_out[:, 2] = normalize(np.cross(rot_out[:, 0], rot_out[:, 1]))

	return rot_out;

################################################################################
if __name__ == "__main__":

	quaternions = np.empty([0, 4], dtype=float)
	positions = np.empty([0, 3], dtype=float)

	with open('/home/mrgribbot/catkin_ws/src/camera_calibration_panda/res/' + sys.argv[1]) as csv_file:

		csv_reader = csv.reader(csv_file, delimiter=',')

		for row in csv_reader:

			tf = parseRow(row)

			pos = np.array(tf[0:3, 3]).reshape(1, 3);
			positions = np.append(positions, pos, axis=0);
			rot_n = normalizeRotation(tf[0:3, 0:3])
			tf[0:3, 0:3] = rot_n;

			quat = pq.Quaternion(matrix=rot_n)
			quat_coeffs = np.array([[quat[0], quat[1], quat[2], quat[3]]]).reshape(1, 4)
			quaternions = np.append(quaternions, quat_coeffs, axis=0)

	avg_quat = averageQuaternions(quaternions)

	pq_avg_q = pq.Quaternion(avg_quat)

	N = quaternions.shape[0]
	squared_diff_sum_quat = 0.0
	squared_diff_sum_pos = np.zeros(3, dtype=float)

	avg_pos = np.zeros(3, dtype=float)

	for i in range(0, N):
		avg_pos += positions[i]

	avg_pos = avg_pos / N

	for i in range(0, N):
		q = pq.Quaternion(quaternions[i])
		squared_diff_sum_quat += math.pow(pq.Quaternion.distance(pq_avg_q, q), 2)

		for j in range(0, 3):
			squared_diff_sum_pos[j] += math.pow(positions[i, j] - avg_pos[j], 2)

	quat_variance = squared_diff_sum_quat / N
	pos_variance = squared_diff_sum_pos / N
	
	print("position variance:", pos_variance)
	print("orientation variance:", quat_variance)
		
