param Nfe := 100;
param Nv := 4;
var tf >= 1;
var dt = tf / Nfe;

param Nobs;
param PPP{i in {1..Nobs}, j in {1..4}, k in {1..2}}; 
param AREA{i in {1..Nobs}};
param BVC {i in {1..(8+Nv)}};
param M{i in {1..Nv}};
 
param amax := 0.25;
param vmax := 1.5;
param wmax := 0.5;
param phymax := 0.7;

param L_tractor_front_hang := 0.25;
param L_tractor_wheelbase := 1.5;
param L_tractor_rear_hang := 0.25;
param LHW := 1;
param L := 3.0;
param L_trailer_front_hang := 1;
param L_trailer_rear_hang := 1;

##### Decision variabes besides tf #####
var x{i in {0..Nfe}, k in {1..Nv}};
var y{i in {0..Nfe}, k in {1..Nv}};
var v{i in {0..Nfe}, k in {1..Nv}};
var theta{i in {0..Nfe}, k in {1..Nv}};
var phy{i in {0..Nfe}};
var AX{i in {0..Nfe}, k in {1..Nv}};
var BX{i in {0..Nfe}, k in {1..Nv}};
var CX{i in {0..Nfe}, k in {1..Nv}};
var DX{i in {0..Nfe}, k in {1..Nv}};
var AY{i in {0..Nfe}, k in {1..Nv}};
var BY{i in {0..Nfe}, k in {1..Nv}};
var CY{i in {0..Nfe}, k in {1..Nv}};
var DY{i in {0..Nfe}, k in {1..Nv}};

############# Minimization objective ###############
minimize objective_:
0.1 * sum{i in {1..Nfe}}((v[i,1]-v[i-1,1])^2 + (phy[i]-phy[i-1])^2) + tf;
 
s.t. time_constraint:
tf <= Nfe * 0.5;


################## ODEs for the towing tractor #############
s.t. DIFF_dxdt {i in {1..Nfe}}:
x[i,1] = x[i-1,1] + dt * v[i-1,1] * cos(theta[i-1,1]);

s.t. DIFF_dydt {i in {1..Nfe}}:
y[i,1] = y[i-1,1] + dt * v[i-1,1] * sin(theta[i-1,1]);

s.t. DIFF_dtheta1dt {i in {1..Nfe}}:
theta[i,1] = theta[i-1,1] + dt * v[i-1,1] * tan(phy[i-1]) / L_tractor_wheelbase;

################## DAEs for the trailers #############
s.t. ALGE_x_2_to_Nv {i in {0..Nfe}, j in {2..Nv}}:
x[i,j] = x[i,j-1] - L * cos(theta[i,j]) - M[j-1] * cos(theta[i,j-1]);

s.t. ALGE_y_2_to_Nv {i in {0..Nfe}, j in {2..Nv}}:
y[i,j] = y[i,j-1] - L * sin(theta[i,j]) - M[j-1] * sin(theta[i,j-1]);

s.t. DIFF_theta_2_to_Nv {i in {1..Nfe}, j in {2..Nv}}:
L * (theta[i,j] - theta[i-1,j]) = dt * (v[i,j-1] * sin(theta[i,j-1] - theta[i,j])) - M[j-1] * cos(theta[i,j-1] - theta[i,j]) * (theta[i,j-1] - theta[i-1,j-1]);

s.t. DIFF_v_2_to_Nv {i in {1..Nfe}, j in {2..Nv}}:
dt * v[i,j] = dt * v[i,j-1] * cos(theta[i,j-1] - theta[i,j]) + M[j-1] * sin(theta[i,j-1] - theta[i,j]) * (theta[i,j-1] - theta[i-1,j-1]);

############# Equations for vertexes A B C D #################### 
s.t. RELATIONSHIP_AX1 {i in {0..Nfe}}:
AX[i,1] = x[i,1] + (L_tractor_front_hang + L_tractor_wheelbase) * cos(theta[i,1]) - LHW * sin(theta[i,1]);

s.t. RELATIONSHIP_BX1 {i in {0..Nfe}}:
BX[i,1] = x[i,1] + (L_tractor_front_hang + L_tractor_wheelbase) * cos(theta[i,1]) + LHW * sin(theta[i,1]);

s.t. RELATIONSHIP_CX1 {i in {0..Nfe}}:
CX[i,1] = x[i,1] - L_tractor_rear_hang * cos(theta[i,1]) + LHW * sin(theta[i,1]);

s.t. RELATIONSHIP_DX1 {i in {0..Nfe}}:
DX[i,1] = x[i,1] - L_tractor_rear_hang * cos(theta[i,1]) - LHW * sin(theta[i,1]);

s.t. RELATIONSHIP_AY1 {i in {0..Nfe}}:
AY[i,1] = y[i,1] + (L_tractor_front_hang + L_tractor_wheelbase) * sin(theta[i,1]) + LHW * cos(theta[i,1]);

s.t. RELATIONSHIP_BY1 {i in {0..Nfe}}:
BY[i,1] = y[i,1] + (L_tractor_front_hang + L_tractor_wheelbase) * sin(theta[i,1]) - LHW * cos(theta[i,1]);

s.t. RELATIONSHIP_CY1 {i in {0..Nfe}}:
CY[i,1] = y[i,1] - L_tractor_rear_hang * sin(theta[i,1]) - LHW * cos(theta[i,1]);

s.t. RELATIONSHIP_DY1 {i in {0..Nfe}}:
DY[i,1] = y[i,1] - L_tractor_rear_hang * sin(theta[i,1]) + LHW * cos(theta[i,1]);

s.t. RELATIONSHIP_AX2toNv {i in {0..Nfe}, k in {2..Nv}}:
AX[i,k] = x[i,k] + L_trailer_front_hang * cos(theta[i,k]) - LHW * sin(theta[i,k]);

s.t. RELATIONSHIP_BX2toN {i in {0..Nfe}, k in {2..Nv}}:
BX[i,k] = x[i,k] + L_trailer_front_hang * cos(theta[i,k]) + LHW * sin(theta[i,k]);

s.t. RELATIONSHIP_CX2toN {i in {0..Nfe}, k in {2..Nv}}:
CX[i,k] = x[i,k] - L_trailer_rear_hang * cos(theta[i,k]) + LHW * sin(theta[i,k]);

s.t. RELATIONSHIP_DX2toN {i in {0..Nfe}, k in {2..Nv}}:
DX[i,k] = x[i,k] - L_trailer_rear_hang * cos(theta[i,k]) - LHW * sin(theta[i,k]);

s.t. RELATIONSHIP_AY2toN {i in {0..Nfe}, k in {2..Nv}}:
AY[i,k] = y[i,k] + L_trailer_front_hang * sin(theta[i,k]) + LHW * cos(theta[i,k]);

s.t. RELATIONSHIP_BY2toN {i in {0..Nfe}, k in {2..Nv}}:
BY[i,k] = y[i,k] + L_trailer_front_hang * sin(theta[i,k]) - LHW * cos(theta[i,k]);

s.t. RELATIONSHIP_CY2toN {i in {0..Nfe}, k in {2..Nv}}:
CY[i,k] = y[i,k] - L_trailer_rear_hang * sin(theta[i,k]) - LHW * cos(theta[i,k]);

s.t. RELATIONSHIP_DY2toN {i in {0..Nfe}, k in {2..Nv}}:
DY[i,k] = y[i,k] - L_trailer_rear_hang * sin(theta[i,k]) + LHW * cos(theta[i,k]);

######## Boundary value conditions ########
s.t. EQ_init_x :
x[0,1] = BVC[1];
s.t. EQ_init_y :
y[0,1] = BVC[2];
s.t. EQ_init_v:
v[0,1] = BVC[3];
s.t. EQ_init_phy :
phy[0] = BVC[4];
s.t. EQ_init_theta_1_to_Nv {k in {1..Nv}}:
theta[0,k] = BVC[4+k];
s.t. EQ_end_v :
v[Nfe,1] = 0;
s.t. EQ_ending_a :
v[Nfe-1,1] = 0;
s.t. EQ_ending_w :
phy[Nfe-1] = phy[Nfe];

s.t. IEQ_ending_AX {k in {1..Nv}}:
BVC[9] - BVC[11] <= AX[Nfe,k] <= BVC[9] + BVC[11];
s.t. IEQ_ending_BX {k in {1..Nv}}:
BVC[9] - BVC[11] <= BX[Nfe,k] <= BVC[9] + BVC[11];
s.t. IEQ_ending_CX {k in {1..Nv}}:
BVC[9] - BVC[11] <= CX[Nfe,k] <= BVC[9] + BVC[11];
s.t. IEQ_ending_DX {k in {1..Nv}}:
BVC[9] - BVC[11] <= DX[Nfe,k] <= BVC[9] + BVC[11];

s.t. IEQ_ending_AY {k in {1..Nv}}:
BVC[10] - BVC[12] <= AY[Nfe,k] <= BVC[10] + BVC[12];
s.t. IEQ_ending_BY {k in {1..Nv}}:
BVC[10] - BVC[12] <= BY[Nfe,k] <= BVC[10] + BVC[12];
s.t. IEQ_ending_CY {k in {1..Nv}}:
BVC[10] - BVC[12] <= CY[Nfe,k] <= BVC[10] + BVC[12];
s.t. IEQ_ending_DY {k in {1..Nv}}:
BVC[10] - BVC[12] <= DY[Nfe,k] <= BVC[10] + BVC[12];


########## Path constraints ##########
s.t. Bonds_v1 {i in {1..Nfe}}:
-vmax <= v[i,1] <= vmax;
s.t. Bonds_a {i in {1..Nfe}}:
-amax <= v[i,1] - v[i-1,1] <= amax;
s.t. Bonds_phy {i in {1..Nfe}}:
-phymax <= phy[i] <= phymax;
s.t. Bonds_w {i in {1..Nfe}}:
-wmax <= phy[i] - phy[i-1] <= wmax;
s.t. Bonds_dtheta {i in {1..Nfe}, k in {2..Nv}}:
-1.5708 <= theta[i,k] - theta[i,k-1] <= 1.5708;

## Environment box ##
s.t. IEQ_entire_time_AY {i in {1..Nfe}, pp in {1..Nv}}:
-20 <= AY[i,pp] <= 20;
s.t. IEQ_entire_time_BY {i in {1..Nfe}, pp in {1..Nv}}:
-20 <= BY[i,pp] <= 20;
s.t. IEQ_entire_time_CY {i in {1..Nfe}, pp in {1..Nv}}:
-20 <= CY[i,pp] <= 20;
s.t. IEQ_entire_time_DY {i in {1..Nfe}, pp in {1..Nv}}:
-20 <= DY[i,pp] <= 20;

s.t. IEQ_entire_time_AX {i in {1..Nfe}, pp in {1..Nv}}:
-30 <= AX[i,pp] <= 20;
s.t. IEQ_entire_time_BX {i in {1..Nfe}, pp in {1..Nv}}:
-30 <= BX[i,pp] <= 20;
s.t. IEQ_entire_time_CX {i in {1..Nfe}, pp in {1..Nv}}:
-30 <= CX[i,pp] <= 20;
s.t. IEQ_entire_time_DX {i in {1..Nfe}, pp in {1..Nv}}:
-30 <= DX[i,pp] <= 20;

## Collison avoidance ##
s.t. eq_PPPoutsideABCD {i in {1..Nfe}, pp in {1..Nv}, nn in {1..Nobs}, jj in {1..4}}:
abs((AX[i,pp] - PPP[nn,jj,1])*(BY[i,pp] - PPP[nn,jj,2]) - (AY[i,pp] - PPP[nn,jj,2])*(BX[i,pp] - PPP[nn,jj,1])) * 0.5 + abs((BX[i,pp] - PPP[nn,jj,1])*(CY[i,pp] - PPP[nn,jj,2]) - (BY[i,pp] - PPP[nn,jj,2])*(CX[i,pp] - PPP[nn,jj,1])) * 0.5 + abs((CX[i,pp] - PPP[nn,jj,1])*(DY[i,pp] - PPP[nn,jj,2]) - (CY[i,pp] - PPP[nn,jj,2])*(DX[i,pp] - PPP[nn,jj,1])) * 0.5 + abs((DX[i,pp] - PPP[nn,jj,1])*(AY[i,pp] - PPP[nn,jj,2]) - (DY[i,pp] - PPP[nn,jj,2])*(AX[i,pp] - PPP[nn,jj,1])) * 0.5 >= 4 + 0.1;

s.t. eq_AoutsidePRECTANGLEPPP {i in {1..Nfe}, pp in {1..Nv}, nn in {1..Nobs}}:
abs((PPP[nn,1,1] - AX[i,pp])*( PPP[nn,2,2] - AY[i,pp]) - (PPP[nn,1,2] - AY[i,pp])*(PPP[nn,2,1] - AX[i,pp])) * 0.5 + abs((PPP[nn,2,1] - AX[i,pp])*(PPP[nn,3,2] - AY[i,pp]) - (PPP[nn,2,2] - AY[i,pp])*( PPP[nn,3,1] - AX[i,pp])) * 0.5 + abs((PPP[nn,3,1] - AX[i,pp])*( PPP[nn,4,2] - AY[i,pp]) - (PPP[nn,3,2] - AY[i,pp])*( PPP[nn,4,1] - AX[i,pp])) * 0.5 + abs((PPP[nn,4,1] - AX[i,pp])*( PPP[nn,1,2] - AY[i,pp]) - (PPP[nn,4,2] - AY[i,pp])*( PPP[nn,1,1] - AX[i,pp])) * 0.5 >= AREA[nn] + 0.1;

s.t. eq_BoutsidePRECTANGLEPPP {i in {1..Nfe}, pp in {1..Nv}, nn in {1..Nobs}}:
abs((PPP[nn,1,1] - BX[i,pp])*( PPP[nn,2,2] - BY[i,pp]) - (PPP[nn,1,2] - BY[i,pp])*(PPP[nn,2,1] - BX[i,pp])) * 0.5 + abs((PPP[nn,2,1] - BX[i,pp])*(PPP[nn,3,2] - BY[i,pp]) - (PPP[nn,2,2] - BY[i,pp])*( PPP[nn,3,1] - BX[i,pp])) * 0.5 + abs((PPP[nn,3,1] - BX[i,pp])*( PPP[nn,4,2] - BY[i,pp]) - (PPP[nn,3,2] - BY[i,pp])*( PPP[nn,4,1] - BX[i,pp])) * 0.5 + abs((PPP[nn,4,1] - BX[i,pp])*( PPP[nn,1,2] - BY[i,pp]) - (PPP[nn,4,2] - BY[i,pp])*( PPP[nn,1,1] - BX[i,pp])) * 0.5 >= AREA[nn] + 0.1;

s.t. eq_CoutsidePRECTANGLEPPP {i in {1..Nfe}, pp in {1..Nv}, nn in {1..Nobs}}:
abs((PPP[nn,1,1] - CX[i,pp])*( PPP[nn,2,2] - CY[i,pp]) - (PPP[nn,1,2] - CY[i,pp])*(PPP[nn,2,1] - CX[i,pp])) * 0.5 + abs((PPP[nn,2,1] - CX[i,pp])*(PPP[nn,3,2] - CY[i,pp]) - (PPP[nn,2,2] - CY[i,pp])*( PPP[nn,3,1] - CX[i,pp])) * 0.5 + abs((PPP[nn,3,1] - CX[i,pp])*( PPP[nn,4,2] - CY[i,pp]) - (PPP[nn,3,2] - CY[i,pp])*( PPP[nn,4,1] - CX[i,pp])) * 0.5 + abs((PPP[nn,4,1] - CX[i,pp])*( PPP[nn,1,2] - CY[i,pp]) - (PPP[nn,4,2] - CY[i,pp])*( PPP[nn,1,1] - CX[i,pp])) * 0.5 >= AREA[nn] + 0.1;

s.t. eq_DoutsidePRECTANGLEPPP {i in {1..Nfe}, pp in {1..Nv}, nn in {1..Nobs}}:
abs((PPP[nn,1,1] - DX[i,pp])*( PPP[nn,2,2] - DY[i,pp]) - (PPP[nn,1,2] - DY[i,pp])*(PPP[nn,2,1] - DX[i,pp])) * 0.5 + abs((PPP[nn,2,1] - DX[i,pp])*(PPP[nn,3,2] - DY[i,pp]) - (PPP[nn,2,2] - DY[i,pp])*( PPP[nn,3,1] - DX[i,pp])) * 0.5 + abs((PPP[nn,3,1] - DX[i,pp])*( PPP[nn,4,2] - DY[i,pp]) - (PPP[nn,3,2] - DY[i,pp])*( PPP[nn,4,1] - DX[i,pp])) * 0.5 + abs((PPP[nn,4,1] - DX[i,pp])*( PPP[nn,1,2] - DY[i,pp]) - (PPP[nn,4,2] - DY[i,pp])*( PPP[nn,1,1] - DX[i,pp])) * 0.5 >= AREA[nn] + 0.1;

data;
param: M := include M;
param Nobs := include Number_obstacle;
param: PPP := include Current_vertex;
param AREA := include Area;
param BVC := include BVC;