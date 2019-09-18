param Nfe := 100;
param Nv := 4;
var tf >= 1;
var dt = tf / Nfe;
param BVC {i in {1..(8+Nv)}};
param Waypoints{i in {0..Nfe}, j in {1..2}};
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

minimize objective_:
1;

s.t. time_constraint:
tf <= Nfe * 0.5;

s.t. DIFF_dxdt {i in {1..Nfe}}:
x[i,1] = x[i-1,1] + dt * v[i-1,1] * cos(theta[i-1,1]);

s.t. DIFF_dydt {i in {1..Nfe}}:
y[i,1] = y[i-1,1] + dt * v[i-1,1] * sin(theta[i-1,1]);

s.t. DIFF_dtheta1dt {i in {1..Nfe}}:
theta[i,1] = theta[i-1,1] + dt * v[i-1,1] * tan(phy[i-1]) / L_tractor_wheelbase;

s.t. ALGE_x_2_to_Nv {i in {0..Nfe}, j in {2..Nv}}:
x[i,j] = x[i,j-1] - L * cos(theta[i,j]) - M[j-1] * cos(theta[i,j-1]);

s.t. ALGE_y_2_to_Nv {i in {0..Nfe}, j in {2..Nv}}:
y[i,j] = y[i,j-1] - L * sin(theta[i,j]) - M[j-1] * sin(theta[i,j-1]);

s.t. DIFF_theta_2_to_Nv {i in {1..Nfe}, j in {2..Nv}}:
L * (theta[i,j] - theta[i-1,j]) = dt * (v[i,j-1] * sin(theta[i,j-1] - theta[i,j])) - M[j-1] * cos(theta[i,j-1] - theta[i,j]) * (theta[i,j-1] - theta[i-1,j-1]);

s.t. DIFF_v_2_to_Nv {i in {1..Nfe}, j in {2..Nv}}:
dt * v[i,j] = dt * v[i,j-1] * cos(theta[i,j-1] - theta[i,j]) + M[j-1] * sin(theta[i,j-1] - theta[i,j]) * (theta[i,j-1] - theta[i-1,j-1]);
 
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

data;
param: M := include M;
param: Waypoints := include Waypoints;
param BVC := include BVC;