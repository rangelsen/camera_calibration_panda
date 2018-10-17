function estimate_camera_pose()

	res_path_prefix = "/home/mrgribbot/Documents/calib-dataset1/";
	local_res = "../res/";
	board_poses 	  = csvread(strcat(local_res, "cTch_ver.csv"));
	endeffector_poses = csvread(strcat(res_path_prefix, "bTe.csv"));

	n_calib_points = size(board_poses, 1)

	AA = nan(4, (n_calib_points - 1) * 4);
	BB = nan(4, (n_calib_points - 1) * 4);

	for i = 1:n_calib_points-1
		
		point_idx0 = board_poses(i, 1);
		point_idx1 = board_poses(i + 1, 1);
		A0 = pose_from_line(board_poses(i, 2:end));
		A1 = pose_from_line(board_poses(i + 1, 2:end));
		A = A1 * inv(A0);
		start_col = (i - 1) * 4 + 1;
		AA(:, start_col:(start_col + 3)) = A;

		B0 = get_pose_by_index(point_idx0, endeffector_poses);
		B1 = get_pose_by_index(point_idx1, endeffector_poses);
		B = B1 * inv(B0);
		BB(:, start_col:(start_col + 3)) = B;
	end

	% cTb = andreff(AA, BB);
	cTb = park(AA, BB);
	% cTb = daniilidis(AA, BB);

	bTc= inv(cTb)

	flat_pose = flatten_pose(bTc);

	csvwrite(strcat(local_res, "bTc.csv"), flat_pose);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function flat_pose = flatten_pose(pose)
	
	flat_pose = nan(1, 17);
	flat_pose(1) = -1;

	for i = 1:4
		for j = 1:4

			flat_pose(1, (j - 1) + (i - 1) * 4 + 2) = pose(i, j);
		end
	end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function pose = get_pose_by_index(index, poses)

	for i = 1:size(poses, 1)

		if poses(i, 1) == index
			
			pose = pose_from_line(poses(i, 2:end));
			break;
		end
	end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [X]=andreff(AA,BB)
% Solves the problem AX=XB
% using the formulation of
%
% On-line Hand-Eye Calibration.
% N. Andreff, R. Horaud, B. Espiau 
%
% Mili Shah
% July 2014

	[m,n]=size(AA); n = n/4;

	A = zeros(12*n,12);
	b = zeros(12*n,1);
	for i = 1:n
		Ra = AA(1:3,4*i-3:4*i-1);
		Rb = BB(1:3,4*i-3:4*i-1);
		ta = AA(1:3,4*i);
		tb = BB(1:3,4*i);
		A(12*i-11:12*i-3,1:9) = eye(9) - kron(Rb,Ra);
		A(12*i-2:12*i,:) = [kron(tb',eye(3)) eye(3)-Ra];
		b(12*i-2:12*i) = ta;
	end
	x = A\b;

	X = reshape(x(1:9),3,3)';
	X = sign(det(X)) / abs(det(X))^(1/3)*X;

	[u,s,v]=svd(X); X = u*v';

	if det(X)<0

		X = u*diag([1 1 -1])*v';
	end

	X = [X' x(10:12);[0 0 0 1]];

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function pose = pose_from_line(line)

	pose = nan(4, 4);

	for j = 1:4

		start_col = (j - 1) * 4 + 1;
		end_col = start_col + 3;

		pose(j, :) = line(start_col:end_col);
	end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function X = park(A, B)

% Calculates the least squares solution of
% AX = XB
% From
% Robot Sensor Calibration: 
% Solving AX=XB on the Euclidean Group
% M. Park
%
% Mili Shah
% July 2014

[m,n] = size(A); n = n/4;
M = zeros(3,3);
a1 = zeros(3,n);
b1 = zeros(3,n);

%Calculate best rotation R
for i = 1:n
    A1 = logm(A(1:3,4*i-3:4*i-1));
    B1 = logm(B(1:3,4*i-3:4*i-1));
    a1(:,i) = [A1(3,2) A1(1,3) A1(2,1)]';
    b1(:,i) = [B1(3,2) B1(1,3) B1(2,1)]';
    M  = M + b1(:,i)*a1(:,i)';
end
[u,v] = eig(M'*M);
v = diag(diag(v).^(-1/2));
R = u*v*u'*M';

%Calculate best translation t
C = zeros(3*n,3);
d = zeros(3*n,1);
I = eye(3);
for i = 1:n
    C(3*i-2:3*i,:) = I - A(1:3,4*i-3:4*i-1);
    d(3*i-2:3*i,:) = A(1:3,4*i)-R*B(1:3,4*i);
end
t = C\d;

%Put everything together to form X
X = [R t;0 0 0 1];

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function X = daniilidis(A, B)
% Calculates the least squares solution of
% AX = XB
%
% The dual quaternion approach to hand-eye calibration
% Daniildis, Bayro-Corrochano
%
% Mili Shah
% July 2014
%
% Uses hom2quar.m and quar2hom.m

[m,n] = size(A); n = n/4;
T = zeros(6*n,8);

for i = 1:n
    A1 = (A(:,4*i-3:4*i));
    B1 = (B(:,4*i-3:4*i));
    a = hom2quar(A1);
    b = hom2quar(B1);
    T(6*i-5:6*i,:) = ...
        [a(2:4,1)-b(2:4,1)  skew(a(2:4,1)+b(2:4,1)) zeros(3,4);...
        a(2:4,2)-b(2:4,2) skew(a(2:4,2)+b(2:4,2)) a(2:4,1)-b(2:4,1)  skew(a(2:4,1)+b(2:4,1))];
end
[u,s,v]=svd(T);
u1 = v(1:4,7);
v1 = v(5:8,7);
u2 = v(1:4,8);
v2 = v(5:8,8);

a = u1'*v1;
b = u1'*v2+u2'*v1;
c = u2'*v2;

s1 = (-b+sqrt(b^2-4*a*c))/2/a;
s2 = (-b-sqrt(b^2-4*a*c))/2/a;

s  = [s1; s2];
[val,in] = max(s.^2*(u1'*u1) + 2*s*(u1'*u2) + u2'*u2);
s = s(in);
L2 = sqrt(1/val);
L1 = s*L2;

q = L1*v(:,7) + L2*v(:,8);
X = quar2hom([q(1:4) q(5:8)]);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function dq = hom2quar(H)
% Converts 4x4 homogeneous matrix H
% to a dual quaternion dq represented as
% dq(:,1) + e*dq(:,2)
%
% Mili Shah

R = H(1:3,1:3);
t = H(1:3,4);

R = logm(R);
r = [R(3,2) R(1,3) R(2,1)]';
theta = norm(r);
l = r/norm(theta);

q = [cos(theta/2); sin(theta/2)*l];
qprime = .5*qmult([0;t],q);
dq=[q qprime];
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function H = quar2hom(dq)
% Converts dual quaternion dq represented as
% dq(:,1) + e*dq(:,2)
% to a 4x4 homogeneous matrix H
%
% Uses the m-file
% qmult.m
%
% Mili Shah

q = dq(:,1); qe = dq(:,2);

R = [1-2*q(3)^2-2*q(4)^2 2*(q(2)*q(3)-q(4)*q(1)) 2*(q(2)*q(4)+q(3)*q(1));...
    2*(q(2)*q(3)+q(4)*q(1)) 1-2*q(2)^2-2*q(4)^2 2*(q(3)*q(4)-q(2)*q(1));...
    2*(q(2)*q(4)-q(3)*q(1)) 2*(q(3)*q(4)+q(2)*q(1)) 1-2*q(2)^2-2*q(3)^2];

q(2:4) = -q(2:4);
t = 2*qmult(qe,q);

H = [R t(2:4);0 0 0 1];
end
