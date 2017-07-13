#Codigo Python para el calculo de las medidas de calidad en agarres
#Codigo portado manualmente desde Matlab

from QualityMeasures import *
from openravepy import *
from numpy import *

def CalculateMeasures(objectid, contactsVector, robotid, fingers, links):
    
    measures=[]  
    
    bb=objectid.ComputeAABB()
    centro_obj=bb.pos()
    object_links=objectid.GetLinkTransformations()
    centro_masa_obj=[]
    for link in object_links:
        for i in range(3):
            centro_masa_obj.append(link[i][3])
    measures.append(A1(contactsVector, centro_obj))
    measures.append(A2(contactsVector, centro_obj))
    measures.append(A3(contactsVector, centro_obj))
    measures.append(B1(contactsVector, centro_masa_obj, bb))
    measures.append(B2(contactsVector, robotid))
    measures.append(B3(contactsVector))
    measures.append(C1(contactsVector, objectid, centro_masa_obj, bb))
    measures.append(C2(contactsVector, objectid, centro_masa_obj, bb, robotid))
    measures.append(D1(robotid))
    measures.append(D2(centro_obj, contactsVector, robotid, fingers, links))    
    
    return measures

# A1 - Valor Singular Minimo
def A1(contacts, centro_obj):
    min_sing_value=None
    if len(contacts)!=0:
        C=CalculateFullG(contacts, centro_obj)
        min_sing_value=MinSV(C)
    return min_sing_value
    
    
# A2 - Volumen Elipsoide
def A2(contacts, centro_obj):
    elipsoide=None
    if len(contacts)!=0:
        C=CalculateFullG(contacts, centro_obj)
        elipsoide=ElipsoidVolume(C)
    return elipsoide
# A3 - Indice de Isotropia
def A3(contacts, centro_obj):
    isotropy_index=None
    if len(contacts)!=0:
        C=CalculateFullG(contacts, centro_obj)
        isotropy_index=IsotropyIndex(C)
    return isotropy_index
    

# B1 - Distancia centroide al centro de masa
def B1(contacts, centro_masa_obj, bb):
    distancia=None
    if len(contacts)>=2:
        puntos=array(contacts).transpose()[0:3].transpose()        
        distancia = DistanciaCentroideCentroMasas(puntos, centro_masa_obj)
        if distancia==None:
            return distancia
           
        #            normalizado
        #            Normalizamos la medida de la distancia
        #            PARA LA BOUNDING BOX
        #            el 3 es el linkid y lo tengo que traer como parametro
        
        vector_maxima_distancia = (array(centro_masa_obj) - array([bb.pos()[0]+bb.extents()[0], bb.pos()[1]+bb.extents()[1], bb.pos()[2]+bb.extents()[2]]))
        maxima_distancia = sqrt(vector_maxima_distancia[0]**2 + vector_maxima_distancia[1]**2 + vector_maxima_distancia[2]**2);
                 
        #            Distancia normalizada para comparar con otros agarres
        distancia = distancia / maxima_distancia;

        #            invierto la medida de la distancia porque queremos que ocurra que
        #            cuando el valor se aproxima a 1 es una calidad buena y cuando se
        #        aproxima a 0 es una calidad mala.
        distancia = 1 - distancia;
        return distancia

    return distancia     
    
# B2 - AREA DEL POLIGONO DE PRENSION    
def B2(contacts, robotid):
#      el area maxima del pentagono de prension se ha calculado por
#      experimentacion con la palama de la mano abierta y el resultado
#      que se ha obtenido es: 0.008729234907930
#     maxArea = 0.008729234907930;
    maxArea=1
    if robotid.GetName()=="BarrettHand":
        maxArea=0.044
    elif robotid.GetName()=="BarrettHand4Fingers":
        maxArea=0.056
    elif robotid.GetName()=="SahHand":
        maxArea=0.047
    elif robotid.GetName()=="SchunkHand":
        maxArea=0.063
    elif robotid.GetName()=="ShadowHand3Fingers":
        maxArea=0.011
    elif robotid.GetName()=="ShadowHand4Fingers":
        maxArea=0.012
    elif robotid.GetName()=="ShadowHand":
        maxArea=0.028
    elif robotid.GetName()=="ModelT":
        maxArea=0.02
    elif robotid.GetName()=="Michelangelo":
        maxArea=0.01
    elif robotid.GetName()=="Pr2Hand":
        maxArea=0.0066
    #maxArea = 0.0052 #Michelangelo Hand
    #maxArea = 0.032 #BarrettHand 3 Fingers
    #maxArea = 0.054 #BarrettHand 4 Fingers
    #maxArea = 0.014 #SAH    
    #maxArea = 0.048 #Schunk
    #maxArea = 0.011 #Shadow 3 Fingers
    #maxArea = 0.012 #Shadow 4 Fingers
    #maxArea = 0.017 #Shadow 5 Fingers
    medida=None
    if len(contacts)>=3:
        puntos=array(contacts).transpose()[0:3].transpose()
        aux = ascontiguousarray(puntos).view(dtype((void, puntos.dtype.itemsize * puntos.shape[1])))
        puntos=unique(aux).view(puntos.dtype).reshape(-1, puntos.shape[1])
        if(len(puntos)>=3):
            areaPoligonoPrension = AreaPoligonoPrension(puntos) 
            medida = areaPoligonoPrension/maxArea           
    return medida


# B3 - FORMA DEL POLIGONO DE PRENSION
def B3(contacts):   
    medida=None
    if len(contacts)>=3:
        #Para que sea un poligono tiene que tener por lo menos 3 lados
        puntos=array(contacts).transpose()[0:3].transpose()
        aux = ascontiguousarray(puntos).view(dtype((void, puntos.dtype.itemsize * puntos.shape[1])))
        puntos=unique(aux).view(puntos.dtype).reshape(-1, puntos.shape[1])
        
        if(len(puntos)>=3):            
            indiceCalidad = FormaPoligonoPrension(puntos)            
            medida = abs(1 - indiceCalidad)
    
    return medida

# C1 - Distancia Minima
def C1(contacts, objectid, centro_masa_obj, bb):
    medida=None
    if(len(contacts)>=3):
        
        C,maxima_distancia=CalculateContactsAndDistance(contacts, objectid, centro_masa_obj, bb)
        
        #FORMA ANTIGUA DE CALCULAR MINDIST         
        #normalizado
        [kfailPlanes, indchfin, disfin, parfacfin] = fctestYN(C, maxima_distancia)    
        mindist = min(disfin)
        if(mindist == -100):
            mindist = None
        else:
            mindist = mindist/sqrt(2)            
        medida = mindist         
    return medida
            

# C2 - VOLUMEN
def C2(contacts, objectid, centro_masa_obj, bb, robotid):
    if(len(contacts)>=3):
    #         VOLUMEN
    #         Para poder ejecutar las funciones se necesitan al menos 7 puntos de
    #         contacto, y como nosotros tenemos solo 5 procedemos a descomponer la
    #         normal que se genera por cada dedo en ocho normales por dedo.
    
        C,maxima_distancia=CalculateContactsAndDistance(contacts, objectid, centro_masa_obj, bb)
        
        #normalizado (el objeto no esta en la posicion (0,0,0) por lo
        #que hay que convertir las medidas a esa posicion)
        vol = Volume(C, maxima_distancia)
        if robotid.GetName()=="Pr2Hand":#4 contacts
            v_max=0.415886684424
        elif "arrett" in robotid.GetName():#6 contacts
            v_max=1.40471200403
        elif robotid.GetName()=="BarrettHand4Fingers":#8 contacts
            v_max=2.1328559822
        elif robotid.GetName()=="SahHand":#12 contacts
            v_max=2.85380545281
        elif robotid.GetName()=="SchunkHand":#6 contacts
            v_max=1.40471200403
        elif robotid.GetName()=="ShadowHand3Fingers":#3 contacts
            v_max=1
        elif robotid.GetName()=="ShadowHand4Fingers":#4 contacts
            v_max=0.415886684424
        elif robotid.GetName()=="ShadowHand":#5 contacts
            v_max=0.76729543734
        elif robotid.GetName()=="ModelT":#8 contacts
            v_max=2.1328559822
        elif robotid.GetName()=="Michelangelo":#5 contacts
            v_max=0.85200737
        else:
            v_max = 0.76729543734 #Monte Carlo 
        if vol==0:
            medida=None
        else:
            vol = vol/v_max 

            #Si no lo quisieramos convertir
            #vol = Volume(C, 1)
            
            
            medida = vol
    else:
        medida = None

    return medida   
    
#D1 - Finger Posture
def D1(robotid):
    #FINGER POSTURE
    #capturamos la postura actual
    mat_postura_actual = robotid.GetDOFValues()
    
    limites = robotid.GetDOFLimits()
    #NATURAL POSTURE
    #posture=[0.0;0.0;0.0;0.0;0.0;0.0];
    posture=hstack(zeros((len(limites[0]), 1)))

    desviacion_angulo_ponderado = FingerJointPositionWeightOur(posture, mat_postura_actual, limites)
    medida = 1 - desviacion_angulo_ponderado

    return medida

#D2 -Uniformity of transformation
def D2(centro_obj, contacts, robotid, dits, links):
    #Uniformity of transformation
    if(len(contacts)<1):
        medida = -1
    else:
        #display('Inside 17 OK')
        HOJ = Calculate_HandObjectJ(centro_obj, contacts, robotid, dits, links)
        uniformity_transformation = IsotropyIndex(HOJ)
        medida = uniformity_transformation

    return medida
        

