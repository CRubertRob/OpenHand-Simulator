function [medidas_norm] = CalculateMeasures(umano_id, id_link_objeto, contacts_vector)

for measure = [1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17]
    %medidas(measure) = CalculateQG(umano_id, contacts_vector, measure, id_link_objeto, medidas_normalizadas);
    if (measure == 1) || (measure == 2)
    	medidas_norm(measure) = CalculateQG(umano_id, contacts_vector, measure, id_link_objeto, 0);
    else
    	medidas_norm(measure) = CalculateQG(umano_id, contacts_vector, measure, id_link_objeto, 1);
    end
end

format('short','')

transpose(medidas_norm);
QA1= medidas_norm(1);
QA2= medidas_norm(2);
QA3= medidas_norm(3);
QB1= medidas_norm(6);
QB2 = medidas_norm(5);
QB3 = medidas_norm(4);
QC1 = medidas_norm(9); 
QC2 = medidas_norm(10); 
%QC3 = -1.0; %CalculoBiomecanico
QD1 = medidas_norm(11);
QD2 = medidas_norm(17); %Arreglar
%QE1 = -1.0; %CalculoBiomecanico
%medidas_norm= [QA1,QA2,QA3,QB1,QB2,QB3,QC1,QC2,QC3,QD1,QD2,QE1];
medidas_norm= [QA1,QA2,QA3,QB1,QB2,QB3,QC1,QC2,QD1,QD2];
medidas_norm = transpose(medidas_norm);


