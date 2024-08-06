function sbs_index = findAvaSBS(v_axis,sbs_axis,v_size,v_qos)
%find the availabel destination SBSs for migration satisfying QoS
gamma=3.6*10^4;

max_dis=gamma./(v_size.*v_qos); %unit:km->m

sbs_index=[];
for i=1:length(sbs_axis)
    if pdist([sbs_axis{i};v_axis],'euclidean')<=max_dis
        sbs_index=[sbs_index,i];
    end
end