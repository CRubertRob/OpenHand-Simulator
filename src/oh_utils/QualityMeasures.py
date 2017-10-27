#Codigo Python con los algoritmos para calcular las Medidas de Calidad
#Codigo portado manualmente desde Matlab

from numpy import *
from itertools import *
from pyhull.convex_hull import ConvexHull
from pyhull import qconvex
from math import *
import time, logging

def Rotatef(angle, x, y, z):
    vector = [x, y, z]
    vector = vector/linalg.norm(vector)

    ax = vector[0]
    ay = vector[1]
    az = vector[2]
    
    b = angle

    c = cos(b)
    ac = 1 - c
    s = sin(b)
   
    RotVector=[]
    RotVector.append(ax * ax * ac + c)
    RotVector.append(ax * ay * ac - az * s)
    RotVector.append(ax * az * ac + ay * s)
    
    RotVector.append(ay * ax * ac + az * s)
    RotVector.append(ay * ay * ac + c)
    RotVector.append(ay * az * ac - ax * s)
    
    RotVector.append(az * ax * ac - ay * s)
    RotVector.append(az * ay * ac + ax * s)
    RotVector.append(az * az * ac + c)  
    RotVector=array(RotVector)
    RotMatrix = RotVector.reshape(3,3,order='F').copy()
    
    return RotMatrix

def RotateContact(normal):
    RotMat=[]
    target_dir = normal/linalg.norm(normal)
    x_axis = transpose([1, 0, 0])
    rot_angle = arccos((target_dir*x_axis)[0])
    if abs(rot_angle) > 1e-8:
        if (cross(target_dir, x_axis) == [0, 0, 0]).all():
            rot_axis = [0, 0, 1]
        else:
            rot_axis = cross(target_dir, x_axis)/linalg.norm(cross(target_dir, x_axis))
#         #rot_axis = abs(rot_axis)
        RotMat = Rotatef(rot_angle, rot_axis[0], rot_axis[1], rot_axis[2])
    else:
        RotMat = array([[1, 0, 0,],[0, 1, 0],[0, 0, 1]])
    return RotMat

def CalculateG(contacto, normal, centro_objeto):
    #CALCULATEG Summary of this function goes here
    #   Detailed explanation goes here
    G=[]
    c = contacto
    p = centro_objeto

    c_p = c - p
    
    rx = c_p[0]
    ry = c_p[1]
    rz = c_p[2]
    
    S = [[0, -rz, ry],[rz, 0, -rx],[-ry, rx, 0]]
    #Rotamos la normal hasta igualarla al ejeX.
    RotMat = RotateContact(normal)
    
    SR = dot(S, RotMat)
    G =[concatenate((RotMat[0,:],[0, 0, 0])),
          concatenate((RotMat[1,:],[0, 0, 0])),
          concatenate((RotMat[2,:],[0, 0, 0])),
          concatenate((SR[0,:], RotMat[0,:])),
          concatenate((SR[1,:], RotMat[1,:])),
          concatenate((SR[2,:], RotMat[2,:]))]
    return G


def CalculateFullG(contacts, centro_obj):    
    #Como se trata de un soft finger contact, hay que multiplicar por la matriz
    #siguiente.
    
    B = matrix([[1, 0, 0, 0],[0, 1, 0, 0],[0, 0, 1, 0],[0, 0, 0, 1],[0, 0, 0, 0],[0, 0, 0, 0]]) 
    for i in range(len(contacts)):
        
        dedo = contacts[i]
        Wi = matrix(CalculateG(dedo[0:3], dedo[3:], centro_obj))    
        Gi = Wi*B   
        if i==0:
            FullG = Gi
        else:
            FullG=hstack([FullG, Gi])
    return FullG

def MinSV(FullGMatrix):
    U, S, Vh = linalg.svd(FullGMatrix)
    minium_singular_value = min(S)
    return minium_singular_value

def ElipsoidVolume(FullGMatrix):
    U, S, V = linalg.svd(FullGMatrix)    
    ##constante k
    k = 1    
    #METODO 1
    #ellipsoid_volume = k * sqrt(det(FullGMatrix*transpose(FullGMatrix)))    
    #METODO 2
    ellipsoid_volume = k * prod(S)
    return ellipsoid_volume

def IsotropyIndex(FullGMatrix):
    U, S, Vh = linalg.svd(FullGMatrix)
    if max(S)==0:
        return 0
    else:
        isotropy_index = min(S)/max(S)     
    return isotropy_index


def CombinaYComprueba(points):
    P=[]
    for fila in range(len(points)):
        P.append([])
    
    #Consideramos importantes los 3 primeros puntos, que son los que pertenecen
    #al dedo pulgar, indice y medio.
     
    #Comprobar que los puntos no estan alineados ( no sean colineales ), en 
    #cuyo caso cogeremos otros.
    #ordenaciones = nchoosek([1:len(points,1)], 3)
    ordenaciones=[]
    for i in combinations(arange(len(points)), 3):
        ordenaciones.append(i)
    for i in range(len(ordenaciones)):
        for j in range(len(points)):
            if j not in ordenaciones[i]:
                aux=list(ordenaciones[i])
                aux.append(j)
                ordenaciones[i]=tuple(aux)
    colineal = 1
    i = 0
    conjunto=[]
    Pa=[]
    for fila in range(len(points)):
            Pa.append([])
    while i < len(ordenaciones):
        for fila in range(len(points)):
            Pa[fila]=points[ordenaciones[i][fila]]
                
        conjunto.append(Pa)
        determinante = linalg.det([conjunto[i][0], conjunto[i][1], conjunto[i][2]])

        if(determinante != 0 ):
            #Si el determinante es distinto de 0 significa que no es colineal
            for fila in range(len(points)):
                P[fila]=conjunto[i][fila]  
            break
     
        i = i + 1
 
    return P  


def DistanciaCentroideCentroMasas(points, centro_masas_obj):
    #Haya la distancia entre el centro de masas del objeto y el centroide del
    #poligono de prension que se forma al coger el objeto
       
    if len(points)==2:
        centroide_poligono_contacto = vstack(points).sum(axis=0)/len(points)
    elif len(points)>2:
        puntos=CombinaYComprueba(points)    
        centroide_poligono_contacto = vstack(puntos).sum(axis=0)/len(puntos)
    else:
        return None
    if len(centroide_poligono_contacto)!=len(centro_masas_obj):
        return None

    #distancia entre los dos puntos
    vector_diferencia = (centro_masas_obj - centroide_poligono_contacto)
    modulo_vector = sqrt(vector_diferencia[0]**2 + vector_diferencia[1]**2 + vector_diferencia[2]**2)
    distancia = modulo_vector

       
    return distancia
    
def ProjectPointOnPlane(point, plane):
    t = (point[0]*plane[0] + point[1]*plane[1] + point[2]*plane[2] + plane[3])/ (plane[0]*plane[0] + plane[1]*plane[1] + plane[2]*plane[2])
    point_projected = [point[0]-plane[0]*t, point[1]-plane[1]*t, point[2]-plane[2]*t]
    return point_projected

def AreaTri3D(v1, v2, v3):    
    a=v2-v1
    b=v3-v1
    area=linalg.norm(cross(a,b))/2
    return area
    
def AreaPoligonoPrension(points):   

    
    puntos=CombinaYComprueba(points)    
    
    #     
    #     #UNA VEZ SE QUE NO SON COLINEALES
    #     #Hayamos la normal del plano que forman los 3 puntos
    try:
        normal = cross(puntos[0]-puntos[1], puntos[0]-puntos[2])
    except Exception as e:
        return -1
    
    #     #Ecuacion del plano tiene la forma Ax +By + Cz +D = 0
    D = -(normal[0]*puntos[0][0] + normal[1]*puntos[0][1] + normal[2]*puntos[0][2])
    plano = hstack((normal,D))
     
    if len(points)>3:
        for i in range(3, len(points)):
            puntos[i]=ProjectPointOnPlane(puntos[i], plano)
            
    area = AreaTri3D(puntos[0],puntos[1],puntos[2])

    #Una vez tengo los puntos en el mismo plano
    if (len(points) > 3):
        for i in range(3, len(points)):
            area = area + AreaTri3D(puntos[0],puntos[i-1],puntos[i])
            
    return area

def CalculaAngulo3D(p1, p2, p3):
    v1 = array(p2) - array(p1)
    v2 = array(p2) - array(p3)    
    angulo = acos( round((v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2]) / (sqrt(v1[0]**2 + v1[1]**2 + v1[2]**2) * sqrt(v2[0]**2 + v2[1]**2 + v2[2]**2)),6))
    #transformo el angulo a grados
    angulo = angulo*180/pi
    
    return angulo    

def FormaPoligonoPrension(points):
    indiceCalidad=0
    puntos=CombinaYComprueba(points)
     
     
    #UNA VEZ SE QUE NO SON COLINEALES
    #Hayamos la normal del plano que forman los 3 puntos
    try:
        normal = cross(puntos[0]-puntos[1], puntos[0]-puntos[2])
    except Exception as e:
        return -1
     
    #     #Ecuacion del plano tiene la forma Ax +By + Cz +D = 0
    D = -(normal[0]*puntos[0][0] + normal[1]*puntos[0][1] + normal[2]*puntos[0][2])
    plano = hstack((normal,D))
     
    ##Ahora hay que transladar los otros dos puntos al plano D
    if len(points)>3:
        for i in range(3, len(points)):
            puntos[i]=ProjectPointOnPlane(puntos[i], plano)

     
    n = len(points)
    
    angulos=[]
    for x in range(n):
        angulos.append(CalculaAngulo3D(puntos[x-1], puntos[x], puntos[(x+1)%n]))
     
    
    anguloInt =  180*(n - 2)/n
    anguloMax = (n - 2)*180
     
    indiceCalidad = 0
    
    for i in range(len(angulos)):
        indiceCalidad = indiceCalidad + abs(angulos[i] - anguloInt)
     
    indiceCalidad = indiceCalidad/anguloMax
    
    return indiceCalidad
    
def SpawnContacts(contacts, mu, N):    
    if(len(contacts) == 0):
        C= []
        return C
    
    if(mu == 0):
        C = contacts
        return C
    
    C = []
    
    for i in range(len(contacts)):
        #Obtengo una matriz de 3xN con las nuevas normales
        
        Norms = getFrictionCone(contacts[i][3:6], mu, N)
        new_points = tile(contacts[i][0:3],(N,1))
        aux = []
        aux = hstack([new_points, Norms])  
        #Anyadimos los resultados a la que sera nuestra matriz final
        C.append(aux)   
    
    return C

def getFrictionCone(normal, mu, N):

    C=[]
    fdeltaang = 2*pi/N
    fang = 0
    vsincos = []
    aux=[]
    
    for i in range(N):
        aux.append(sin(fang))
        aux.append(cos(fang))
        vsincos.append(aux)
        fang = fang + fdeltaang
        aux=[]
        
    target_dir = normal/linalg.norm(normal)
    z_axis = transpose([0, 0, 1])
    rot_angle = arccos(dot(target_dir,z_axis))
    
    if abs(rot_angle) > 1e-8:        
        rot_axis = cross(target_dir, z_axis)/linalg.norm(cross(target_dir, z_axis))
        RotMat = Rotatef(rot_angle, rot_axis[0], rot_axis[1], rot_axis[2])    
        
        right = array([RotMat[0][0], RotMat[1][0], RotMat[2][0]])
        up = array([RotMat[0][1], RotMat[1][1], RotMat[2][1]])
    else:
        right = 1
        up = 1

    new_normals = []
    for i in range(N):
        C.append((normal + mu*vsincos[i][0]*right + mu*vsincos[i][1]*up)/linalg.norm(normal + mu*vsincos[i][0]*right + mu*vsincos[i][1]*up))     
        
    return C

def CalculateContactsAndDistance(contacts, objectid, centro_masa_obj, bb):
    #DISTANCIA MINIMA    
    #Normalizar los contactos a coordenadas relativas del objeto
    transformHandPos = objectid.GetTransform()                   
    transformHandPosINV = linalg.inv(transformHandPos)
    transformHandNorm = vstack([transformHandPos[0][0:3], transformHandPos[1][0:3],transformHandPos[2][0:3]])       
    transformHandNormINV = linalg.inv(transformHandNorm)
                 
    contactsNew = [];
    for i in range(len(contacts)):
        dedoPos =  hstack([contacts[i][0:3], 1])
        newPos = diag(transformHandPosINV*dedoPos)
        
        dedoNorm = contacts[i][3:6]
        newNorm = diag(transformHandNormINV*dedoNorm)

        contactsNew.append(hstack([newPos[:][0:3],newNorm]))
   
    vector_maxima_distancia = (array(centro_masa_obj) - array([bb.pos()[0]+bb.extents()[0], bb.pos()[1]+bb.extents()[1], bb.pos()[2]+bb.extents()[2]]))
    maxima_distancia = sqrt(vector_maxima_distancia[0]**2 + vector_maxima_distancia[1]**2 + vector_maxima_distancia[2]**2);
    
    C = SpawnContacts(contactsNew, 0.8, 8)
    
    return (C, maxima_distancia)  

def fctestYN(contacts, dmax):    
    #Primitive wrenches
    #primw = transpose([contacts(4:6,:) cross(contacts(1:3,:), contacts(4:6,:))])
    #Primitive wrenches normalized
    contacts=array(contacts).transpose()
    primw = hstack([vstack(contacts[3:].transpose()), vstack((cross(vstack(contacts[0:3].transpose()), vstack(contacts[3:].transpose())))/dmax)])

    #Convex hull of the primitive wrenches
    indch=convhull_local(primw)	
    #indch=qconvex('Qx Pp', primw)
    #indch=mlab.convhulln(primw)
    if indch==None:
       kfailPlanes=100
       indchfin=0
       disfin=-100
       parfacfin=0
       korifac=100
       
    else:
        #Parameters for each of the facets of the CH
        #and distance between the centroid and the origin
        korifac=0
        kcentor=0
        countfacgood=1
        pfacet=[]
        dis=[]
        facgood=[]
        parfac=[]
        for indexFacet in range(len(indch.vertices)):
            for i in range(6):
                #pfacet contains all of the points in each facet j
                #of the CH (points in R6)           
                if indch.vertices[indexFacet][i]<len(primw):
                    pfacet.append(primw[indch.vertices[indexFacet][i]])
            #Parameters of the hyperplane containing the facet j
            [N,D]=fitplane(pfacet)            
            #Distance between the hyperplane j and the origin
            #If a facet comes degenerate, it is simply ignored
            if D!=-100:
                facgood.append(indexFacet)
                #Matrix with the parameters of the hyperplanes containing each facet
                parfac.append(hstack([hstack(N.transpose()), array(-D)]))                                     
                if abs(D)<=1e-6:
                    #Detection of facets through the origin
                    korifac=korifac+1
                    dis.append(0)
                else:
                    dis.append(linalg.norm(D)/linalg.norm(N))
            pfacet=[]
            
        
        #Update of indch
        indch.vertices=array(indch.vertices)[facgood]    
     
    #If there is at least one facet through the origin, then the grasp is not FC
    if korifac==0:
       try:
	       #Find repeated planes
	       diffparfac=diff(parfac, n=1, axis=0)
	       diffparfac=abs(diffparfac)
	       sumdiffparfac=diffparfac.sum(axis=1)
	       facgood=hstack(where(sumdiffparfac>1e-4)).tolist()
	       #facgood=[1, facgood+1]
	       #Eliminate repeated planes
	       parfacfin=array(parfac)[facgood]
	       indchfin=indch.vertices[facgood]
	       disfin=array(dis)[facgood]
	       #Determine if the grasp is FC
	       #CH Centroid
	       centr=hstack([hstack(primw.mean(axis=0)), 1])
	       #Vector whose sign gives the semiplane of the centroid
	       #with respect to the facets of the CH
	       vecsigp=dot(parfacfin,centr.transpose())
	       #Counter for facets with the origin and centroid in different sides
	       kcentor=0
	       #Proof to verify if the origin is inside the CH
	       #Verification of concordance in the signs
		      
	       #zeros((szR,1))              
	       signPosCent=zeros((len(parfacfin),1))
	       for i in range(len(parfacfin)):
		   if sign(vecsigp[i])==sign(parfacfin[i][6]):
		       #ubicorch[i]=1 #Para que sirve esto?
		       pass
		   else:
		       kcentor=kcentor+1             
		 
	       kfailPlanes=kcentor
       except:
	       print "error from QHull"
	       kfailPlanes=korifac
	       indchfin=0
	       disfin=[-100]
	       parfacfin=0

    else:
       kfailPlanes=korifac
       indchfin=0
       disfin=[-100]
       parfacfin=0

    return (kfailPlanes, indchfin, disfin, parfacfin)

     
def convhull_local(data):
    try:
       hull = ConvexHull(data, joggle=False)
       #output = qconvex("i QR0", data)
       output = qconvex('i Pp Qt Qs', data)
       output.pop(0)
       hull.vertices=[[int(i) for i in row.strip().split()] for row in output]
       
    except Exception as e:        
        print "Exception: " + str(e)
        hull = None
    except:
	print "Unknown Exception in Convex Hull"
        hull = None
    #else:	
    #    print "Error in Convex Hull"
    #    hull = None
    return hull
     
     
#     ############################################
#     ############################################
#     ############################################
#     ############################################

def fitplane(x):
    # FITPLANE Fitting of a plane or hyperplane to a set
    # of points.
    # [N,D] = FITPLANE(X,Y,Z) Calculates a least
    # squares fit to the normal N to a plane through
    # a set of points with coordinates X,Y,Z in the
    # form N(1)*X+N(2)*Y+N(3)*Z = D.
    # Normally N is normalized so that D = 1 unless
    # it is close to zero (a plane goes near
    # coordinate system origin).
     
    # [N,D] = FITPLANE(X,Y) calculates a similar fit
    # to a line, while
    # [N,D] = FITPLANE(R) calculates a fit to a
    # hyper-plane. In this case each row of R
    # must correspond to coordinates of a certain
    # point in a set, such as
    # R = [x1 y1 z1 t1 x2 y2 z2 t2 ...].
    # Copyright (c) 1995 by Kirill K. Pankratov
    # kirill@plume.mit.edu
    # 02/09/95
    #Modified by Maximo Roa, maximo.roa@upc.edu
    #to avoid an error message if there are infinite solutions
     
    tol = .00001
    # Tolerance for canonical form reduction
    # Handle input ............................

    R = array(x)     
     
    # Auxillary
    szR = len(R)
    o = ones((szR,1))
    # Make offset (so that plane will not go through
    # the origin)
    r0 = 2*R.min(0)-R.max(0)-1
    R = R-r0
    #There are problems if rank(R)==5
    # Now the fit itself ......................
    if linalg.matrix_rank(R)==6:
        n = linalg.solve(R,o)
        d = 1+dot(r0,n)
        # Compensate offset
        # Try to reduce it to the canonical form
        # (Nx*X + Ny*Y + Nz*Z = 1) if possible
        if abs(d) > tol:
            n = n/d
            d = 1
         
    else:
        #If there is a degeneracy (infinte solutions),
        #the parameter d is arbitrarily set to -100
        #to detect it in a later step
        #The parameters for the hyperplane are set to 0
        n=zeros((szR,1))
        d=-100
     
    return (n,d)
    
def Volume(contacts, dmax):

    #disp('AnalyzeGrasp')
    mindist = 0;
    volume = 0;
    
    # find the screw coordinates
    contacts=array(contacts).transpose()
    S = hstack([vstack(contacts[3:].transpose()), vstack((cross(vstack(contacts[0:3].transpose()), vstack(contacts[3:].transpose())))/dmax)])
    new_S=[]
    for index in S:
        if ~isnan(sum(index))==-1:  
            new_S.append(index)
    S=new_S        
    #need at least 7 contact wrenches to have force closure in 3D
    if(len(S) < 6 or linalg.matrix_rank(S) < 5):
        return 0
    
    hull = qconvex('Pp FS', S)
    if len(hull)>1:
        volume=float(hull[1].split()[2])
    else:
        volume=0
    
    return volume

def FingerJointPositionWeightOur(posicion_inicial, posicion_final, limites): 
    # La variable limites contiene una matriz de 5 filas y 12 columnas que
    # tiene que estar organizada de la siguiente forma:
    # cada fila corresponde a un dedo y cada columna con el movimiento de una
    # join: CMC MCP ( IP || PIP DIP ). Dentro de cada joint pueden haber los
    # siguientes movimientos:
        #CMC, MCP (Extension, Flexion, Adduccion, Abduccion) 
        # IP, PIP, DIP (Extension, Flexion)
    
        
    #Calculamos una matriz con los grados de libertad que tiene cada joint para
    #cada uno de los dedos.
    dof=[]
    max_limits=[]
    min_limits=[]
    for i in range(len(limites[0])):    
        dof.append(max(limites[0][i],limites[1][i]) - min(limites[0][i],limites[1][i]))
        max_limits.append(max(limites[0][i],limites[1][i]))
        min_limits.append(min(limites[0][i],limites[1][i]))
    
    
    posicion_final = around(posicion_final*100000)/100000;
    calidad = 0;
    numero_joints = 0;
    
    for i in range(len(posicion_inicial)):
       #Si la matriz dof tiene un cero en la posicion i,j significa que no
       #tiene grados de libertad en esa posicion y por la tanto no es un
       #joint.
       if (dof[i]!=0):           
           numero_joints = numero_joints + 1
           desplazamiento = posicion_final[i] - posicion_inicial[i]         
           if( posicion_final[i] < posicion_inicial[i] ):           
               calidad = calidad +  (desplazamiento/(posicion_inicial[i] - min_limits[i]))**2
           elif( posicion_final[i] > posicion_inicial[i] ):                       
               calidad = calidad +  (desplazamiento/(posicion_inicial[i] - max_limits[i]))**2
           else:
               calidad = calidad+1
            
    #Hay que dividir la calidad entre el numero de joints
    #El numero de joints se obtiene contando el numero de elementos que no son 
    #cero de la matriz dof. 
    calidad = calidad / numero_joints;    
    return calidad   

def CalculateJ(robotid, linkid, contacto, normal):
    J=[]
     
     
    act=robotid.CalculateActiveJacobian(linkid, contacto)
    ang=robotid.CalculateActiveAngularVelocityJacobian(linkid)
    Z=vstack([act, ang])
    #Matriz de rotacion
    RotMat = RotateContact(normal)
     
    #Contruyo matriz 6x6 para poder multiplicar
    R =[concatenate((RotMat[0,:],[0, 0, 0])),
          concatenate((RotMat[1,:],[0, 0, 0])),
          concatenate((RotMat[2,:],[0, 0, 0])),
          concatenate(([0, 0, 0], RotMat[0,:])),
          concatenate(([0, 0, 0], RotMat[1,:])),
          concatenate(([0, 0, 0], RotMat[2,:]))]   
     
    #Multiplico Z por la matriz de rotacion
    J = dot(transpose(R),Z)
     
    return J


def Calculate_HandObjectJ(centro_obj, contacts, robotid, dits, links):
    #CALCULATE_HANDOBJECTJ Summary of this function goes here
    #   Detailed explanation goes here
    HandObjectJacobian=0
    
    n_c = len(contacts)
    HandJ=[]
    for i in range(len(dits)):
        aux=CalculateJ(robotid, links[i],contacts[i][:3], contacts[i][3:])
        HandJ.append(aux)
#         index=aux[0].nonzero()[0]
#         ini=(i-1)*6+1
#         fin=ini+5
#         for j in range(len(index)):
#             pass
            #HandJ(ini:fin,index(j))=aux(:,index(j))
# 
#     
    # Como los contactos que se realizan son contactos blandos (soft
    # finger) deberemos multiplicar la matriz jacobiana de la mano
    # por la matriz H que por definicion se crea como a continuacion

    HandJ=vstack(HandJ)
    
    B = [[1, 0, 0, 0, 0, 0],         
         [0, 1, 0, 0, 0, 0],
         [0, 0, 1, 0, 0, 0],
         [0, 0, 0, 1, 0, 0]]   
    H=zeros((n_c*4, n_c*6))
    for i in range(n_c):
        for x in range(4):
            aux=H[i*4:(i+1)*4]
            aux[x][i*6:(i+1)*6]=B[x]
        H[i*4:(i+1)*4]=aux
        
        
        
    HandJ_SoftFinger = dot(H,HandJ)
    # Jacobiano mano-objeto
    G = CalculateFullG(vstack(contacts), centro_obj)
    HandObjectJacobian = transpose(linalg.pinv(G)) * HandJ_SoftFinger
    
    return HandObjectJacobian
