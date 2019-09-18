param Nfe := 100;
param Nv := 4;
var tf >= 1;

param L_tractor_front_hang := 0.25;
param L_tractor_wheelbase := 1.5;
param L_tractor_rear_hang := 0.25;
param LHW := 1;
param M{i in {1..Nv}};
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
1;

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

data;
param: M :=
1	1.5
2	1.5
3	1.5
4	1.5;