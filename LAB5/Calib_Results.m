% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly executed under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%

%-- Focal length:
          All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/
fc = [ 3042.231333043818267 ; 3044.353690870445007 ];

%-- Principal point:
cc = [ 2018.675480715948197 ; 926.333758945551381 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ 0.172976583798854 ; -0.791431600505336 ; -0.002390850604200 ; 0.001451460085490 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 9.916797093688285 ; 10.449686982053754 ];

%-- Principal point uncertainty:
cc_error = [ 17.774931334227968 ; 15.167603500481265 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.018318651181428 ; 0.133268594310538 ; 0.002364267313338 ; 0.002969767780091 ; 0.000000000000000 ];

%-- Image size:
nx = 4000;
ny = 1800;


%-- Various other variables (may be ignored if you do not use the Matlab Calibration Toolbox):
%-- Those variables are used to control which intrinsic parameters should be optimized

n_ima = 20;						% Number of calibration images
est_fc = [ 1 ; 1 ];					% Estimation indicator of the two focal variables
est_aspect_ratio = 1;				% Estimation indicator of the aspect ratio fc(2)/fc(1)
center_optim = 1;					% Estimation indicator of the principal point
est_alpha = 0;						% Estimation indicator of the skew coefficient
est_dist = [ 1 ; 1 ; 1 ; 1 ; 0 ];	% Estimation indicator of the distortion coefficients


%-- Extrinsic parameters:
%-- The rotation (omc_kk) and the translation (Tc_kk) vectors for every calibration image and their uncertainties

%-- Image #1:
omc_1 = [ -2.093377e+00 ; -2.054267e+00 ; -5.669834e-01 ];
Tc_1  = [ -1.027668e+03 ; -1.289401e+03 ; 7.528849e+03 ];
omc_error_1 = [ 5.052579e-03 ; 5.784782e-03 ; 1.071110e-02 ];
Tc_error_1  = [ 4.453565e+01 ; 3.794513e+01 ; 2.958003e+01 ];

%-- Image #2:
omc_2 = [ -1.980273e+00 ; -2.010433e+00 ; -5.428304e-01 ];
Tc_2  = [ -1.058663e+03 ; -5.645213e+02 ; 6.070827e+03 ];
omc_error_2 = [ 3.813815e-03 ; 5.294992e-03 ; 9.193430e-03 ];
Tc_error_2  = [ 3.553077e+01 ; 3.045720e+01 ; 2.281435e+01 ];

%-- Image #3:
omc_3 = [ -2.171107e+00 ; -2.198825e+00 ; -2.592510e-01 ];
Tc_3  = [ -7.554341e+02 ; -2.559049e+02 ; 5.581684e+03 ];
omc_error_3 = [ 4.425955e-03 ; 5.863009e-03 ; 1.144312e-02 ];
Tc_error_3  = [ 3.259193e+01 ; 2.784654e+01 ; 2.078581e+01 ];

%-- Image #4:
omc_4 = [ -2.698089e+00 ; -7.635587e-01 ; -6.383935e-01 ];
Tc_4  = [ -4.490967e+02 ; 3.594742e+02 ; 4.629274e+03 ];
omc_error_4 = [ 5.054060e-03 ; 3.682850e-03 ; 9.036335e-03 ];
Tc_error_4  = [ 2.714093e+01 ; 2.282581e+01 ; 1.727100e+01 ];

%-- Image #5:
omc_5 = [ -2.697829e+00 ; -7.125537e-01 ; -1.040070e+00 ];
Tc_5  = [ -8.883575e+01 ; 4.922144e+02 ; 4.223461e+03 ];
omc_error_5 = [ 4.901312e-03 ; 4.442684e-03 ; 8.941895e-03 ];
Tc_error_5  = [ 2.481092e+01 ; 2.084342e+01 ; 1.662931e+01 ];

%-- Image #6:
omc_6 = [ 2.179019e+00 ; 2.128128e+00 ; -8.517030e-02 ];
Tc_6  = [ -2.973002e+02 ; -1.205979e+03 ; 5.684071e+03 ];
omc_error_6 = [ 5.476108e-03 ; 5.168641e-03 ; 1.141706e-02 ];
Tc_error_6  = [ 3.347231e+01 ; 2.826593e+01 ; 2.078997e+01 ];

%-- Image #7:
omc_7 = [ 2.083094e+00 ; 2.036386e+00 ; -2.724458e-01 ];
Tc_7  = [ -3.047094e+02 ; -1.140984e+03 ; 6.261795e+03 ];
omc_error_7 = [ 4.773875e-03 ; 4.999291e-03 ; 9.715167e-03 ];
Tc_error_7  = [ 3.675783e+01 ; 3.109149e+01 ; 2.040443e+01 ];

%-- Image #8:
omc_8 = [ 1.952563e+00 ; 1.864194e+00 ; -4.750350e-01 ];
Tc_8  = [ -3.927906e+02 ; -8.763538e+02 ; 6.959833e+03 ];
omc_error_8 = [ 4.202990e-03 ; 4.918158e-03 ; 8.160631e-03 ];
Tc_error_8  = [ 4.070413e+01 ; 3.455859e+01 ; 2.113897e+01 ];

%-- Image #9:
omc_9 = [ 1.836512e+00 ; 1.717445e+00 ; -6.372571e-01 ];
Tc_9  = [ -4.563087e+02 ; -5.295538e+02 ; 7.497827e+03 ];
omc_error_9 = [ 4.071611e-03 ; 4.992179e-03 ; 7.372094e-03 ];
Tc_error_9  = [ 4.376083e+01 ; 3.726418e+01 ; 2.221635e+01 ];

%-- Image #10:
omc_10 = [ -1.734552e+00 ; -1.755858e+00 ; -8.347722e-01 ];
Tc_10  = [ -2.471578e+02 ; -8.239899e+02 ; 4.338381e+03 ];
omc_error_10 = [ 3.181065e-03 ; 5.365337e-03 ; 7.283585e-03 ];
Tc_error_10  = [ 2.555309e+01 ; 2.168345e+01 ; 1.663641e+01 ];

%-- Image #11:
omc_11 = [ -1.629604e+00 ; -1.651394e+00 ; -9.522163e-01 ];
Tc_11  = [ -2.142856e+02 ; -4.201774e+02 ; 3.990474e+03 ];
omc_error_11 = [ 3.390222e-03 ; 5.582403e-03 ; 6.861946e-03 ];
Tc_error_11  = [ 2.333913e+01 ; 1.987090e+01 ; 1.522791e+01 ];

%-- Image #12:
omc_12 = [ -1.765164e+00 ; -1.821011e+00 ; -7.629768e-01 ];
Tc_12  = [ -6.978481e+01 ; -8.344853e+01 ; 5.358538e+03 ];
omc_error_12 = [ 3.053013e-03 ; 5.814626e-03 ; 7.849926e-03 ];
Tc_error_12  = [ 3.123253e+01 ; 2.654840e+01 ; 1.908720e+01 ];

%-- Image #13:
omc_13 = [ -1.639953e+00 ; -1.717353e+00 ; -8.871218e-01 ];
Tc_13  = [ -4.680551e+01 ; 1.641100e+02 ; 5.114722e+03 ];
omc_error_13 = [ 3.238203e-03 ; 5.902102e-03 ; 7.279578e-03 ];
Tc_error_13  = [ 2.984352e+01 ; 2.534350e+01 ; 1.836487e+01 ];

%-- Image #14:
omc_14 = [ 2.090240e+00 ; 1.813399e+00 ; 1.220501e+00 ];
Tc_14  = [ -3.651590e+02 ; -1.406781e+02 ; 5.344226e+03 ];
omc_error_14 = [ 6.658352e-03 ; 3.001911e-03 ; 7.924073e-03 ];
Tc_error_14  = [ 3.129535e+01 ; 2.661704e+01 ; 2.152570e+01 ];

%-- Image #15:
omc_15 = [ 2.139761e+00 ; 1.639181e+00 ; 7.540441e-01 ];
Tc_15  = [ -8.542586e+02 ; -3.967720e+02 ; 5.888718e+03 ];
omc_error_15 = [ 5.649965e-03 ; 3.254152e-03 ; 7.928155e-03 ];
Tc_error_15  = [ 3.471367e+01 ; 2.941778e+01 ; 2.384139e+01 ];

%-- Image #16:
omc_16 = [ -1.659847e+00 ; -1.958548e+00 ; 6.124602e-02 ];
Tc_16  = [ -1.166726e+03 ; -6.059761e+02 ; 6.221040e+03 ];
omc_error_16 = [ 4.148359e-03 ; 5.254662e-03 ; 7.829498e-03 ];
Tc_error_16  = [ 3.650968e+01 ; 3.118970e+01 ; 2.093629e+01 ];

%-- Image #17:
omc_17 = [ -1.415483e+00 ; -1.770805e+00 ; 2.825899e-01 ];
Tc_17  = [ -7.224318e+02 ; -7.951463e+02 ; 6.626847e+03 ];
omc_error_17 = [ 4.291268e-03 ; 5.200717e-03 ; 6.369374e-03 ];
Tc_error_17  = [ 3.877479e+01 ; 3.303735e+01 ; 2.110979e+01 ];

%-- Image #18:
omc_18 = [ -1.739378e+00 ; -2.104423e+00 ; 2.740819e-01 ];
Tc_18  = [ -1.258549e+03 ; -5.977607e+02 ; 6.602422e+03 ];
omc_error_18 = [ 4.747941e-03 ; 5.519536e-03 ; 8.505224e-03 ];
Tc_error_18  = [ 3.883545e+01 ; 3.316484e+01 ; 2.186812e+01 ];

%-- Image #19:
omc_19 = [ -1.815474e+00 ; -2.295106e+00 ; 5.797399e-01 ];
Tc_19  = [ -1.047110e+03 ; -6.277086e+02 ; 7.152282e+03 ];
omc_error_19 = [ 5.451835e-03 ; 5.443920e-03 ; 8.839998e-03 ];
Tc_error_19  = [ 4.183015e+01 ; 3.581584e+01 ; 2.258555e+01 ];

%-- Image #20:
omc_20 = [ 1.874896e+00 ; 2.413536e+00 ; -7.474973e-01 ];
Tc_20  = [ -9.421990e+02 ; -5.324841e+02 ; 7.396814e+03 ];
omc_error_20 = [ 3.444217e-03 ; 6.418545e-03 ; 9.283316e-03 ];
Tc_error_20  = [ 4.312848e+01 ; 3.699687e+01 ; 2.269920e+01 ];

