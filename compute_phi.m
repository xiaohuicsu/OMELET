function phi=compute_phi(v_axis,sbs_axis,vehicle_sinfo)
    gamma=3.6*10^4;
      
    count=0;
    for i=1:length(v_axis)
        vehicle_id=v_axis{i}{1};
        task_size=vehicle_sinfo.(vehicle_id).v_size;
        s_position=sbs_axis{vehicle_sinfo.(vehicle_id).s_mig};
        v_position=[v_axis{i}{2},v_axis{i}{3}];
        distance= pdist([s_position;v_position],'euclidean');%unit:m->km
        QoS=gamma/(task_size.*distance);
        if QoS<vehicle_sinfo.(vehicle_id).qos
            count=count+1;
        end
    end
    phi=count/length(v_axis);
end

