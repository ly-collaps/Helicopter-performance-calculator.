import math
#ISA0 AT SEA LEVEL 
density_ISA0 = 1.225
T_ISA0 = 288.15
# P_eng_ISA0 =

n_rotors = 18
n_blades = 2 #Number of blades

#Parameters at 5000ft 
density_5k = 16.01846 * 0.0659
T_5k= 288.15 -(0.0065*5000/3.28084) #278.24
P_5k = 101325*(((1-((0.0065*5000))/(3.28084*288.15)))**(9.81/(287.058*0.0065)))
Rau_5k = P_5k/(287.058*T_5k) # =1.055616514

#VTOL Parameters
Cd =0.4 #Assumed drag coefficient of the fuselage
MTOM = 3000/18 #n_rotors
effeciency = 0.75 #effeciency between 0.65-0.85
effeciency_MRD = 0.75 #effeciency main rotor drive 
rpm = 5000 #Rotor RPM [1/min]
c_blade = 0.3 # Main Blade width
r_MR= 0.65 #Radius of main rotor
x0 = 0.1 #Main Rotor head radius
k = 1/math.sqrt(2*density_ISA0) #atmospheric correction factor
A_fus = 0 #No projected area 
g= 9.81


B = 1- (c_blade/(2*r_MR)) #Correction factor
r_MR_corr = B * r_MR
A_MR_corr = math.pi * ( (r_MR_corr**2) -(x0**2))  #Main rotor corrected area 

S_star_MTOM = MTOM*g*A_fus*Cd/ A_MR_corr #Fuselage Slip stream compensation
S_MTOM = (MTOM*g) + S_star_MTOM #Slip straim admission
sigma_5k= density_5k / density_ISA0 #Density ratio

#------------------------------Rotor required Power at 5000ft
P_req_5k = S_MTOM * k *math.sqrt(S_MTOM/A_MR_corr) /(effeciency*math.sqrt(sigma_5k))
print ("Required rotor power at 10000 ft= ",math.floor(P_req_5k),"W" , "= ", math.floor(P_req_5k)/1000,"kW")
P_req_tot_5k = P_req_5k* n_rotors
print("Total equired power at 10000 ft= ",math.floor(P_req_tot_5k),"W","= ", math.floor(P_req_tot_5k)/1000,"kW" )


Vz = 5.08 #Vertical velocity = rate of climb = 1000ft/min

Cl_alpha = 6.4 #Linear lift increase
#------------------------------Induced velocity 
V_i0 = math.sqrt(MTOM*g /(2* density_5k*A_MR_corr))
#------------------------------Inflow ratio 
Vi_toVi0 = -(Vz/(2*V_i0)) + math.sqrt(((Vz/(2*V_i0))**2) +1)
Vi = V_i0 *Vi_toVi0
#------------------------------ Alpha check
omega_MR = rpm*2*math.pi/60
lamda = (Vz + Vi) /( omega_MR * r_MR_corr)
C_TMR = S_MTOM / (A_MR_corr*density_5k*((omega_MR*r_MR_corr)**2))
phi = math.atan(lamda) *180/math.pi

sigma_MR_4blades = n_blades*c_blade*(r_MR_corr-x0)/A_MR_corr #Main rotor density
theta_blades = 3* ( (2*C_TMR/(Cl_alpha * sigma_MR_4blades)) + (lamda/2)) *180/math.pi
alpha_blades = theta_blades - phi
print("Angle of attack with", n_blades ,  "blades is: ", alpha_blades, "deg")
