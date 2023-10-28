# Assumindo valores recebidos: 
#   - Vcar -> Vetor velocidade do carro
#   - wRoda -> velocidade angular da roda
#   - delta -> angulo da roda do carro
#   - xcar, ycar -> posicao do carro
#   - Psi -> angulo entre o carro e o plano de referencia global
#   - dtPsi -> velocidade angular do carro
#   - Track -> array [ [x0,y0,V0], ... ,[xn,yn,Vn] ] pontos da pista

import numpy as np

def get_Vxref(Vcar, Psi):
    return np.linalg.norm(Vcar)*np.cos(Psi)

def get_dtPsiref(Track,xcar,ycar):

def get_Vx(Vcar,delta)
    return np.linalg.norm(Vcar)*np.cos(delta)

def get_slipRatio(Vcar,w,delta):
    # raio da roda do carro
    R=1

    Vx = get_Vx(Vcar,delta)
    return (w*R-Vx)/Vx

def ctrlLongitudinal(Vcar, delta, Psi, w, dtPsiref, dtPsi, roda):
    # ganhos rodas frontais, rodas traseiras, velocidade em X, velocidade angular
    Kkf, Kkr, Kvx, KdtPsi = 250.0, 350.0, 0.1, 0.03 

    Ki = get_slipRatio(Vcar,w,delta)
    Vxref = get_Vxref(Vcar, Psi)
    Vx = get_Vx(Vcar,delta)
    Kref = Kvx*(Vxref-Vx)
    Kdiff = KdtPsi*(dtPsiref-dtPsi)

    if(roda==1 or roda==2):
        Kk=Kkf
    else:
        Kk=Kkr

    return Kk*(Kref-Ki-(-1)**(roda)*Kdiff)

def get_Posref(Track,xcar,ycar):


def get_ePsi(tang, Vcar):
    return np.arcsin(np.cross(tang,Vcar)[2]/(np.linalg.norm(tang)*np.linalg.norm(Vcar)))

def get_ey(Posref, tang):
    return np.cross(Posref,tang)[2]/np.linalg.norm(tang)

def ctrlLateral(ePsi, ey, Vcar):
    k = 1 # ganho
    return ePsi+np.atan(k*ey/np.linalg.norm(Vcar))