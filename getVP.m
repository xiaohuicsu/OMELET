function [v_position,v_sinfo]=getVP(vehicle_data,v_sinfo,t)

v_position={};

v_id_set=fieldnames(v_sinfo);
for k=1:length(v_id_set)
    v_name=v_id_set{k};
    if length(vehicle_data.(v_name).x)>=t
        %{[vehicle_id, vehicle.x_axis, vehicle.y_axis]}
        if vehicle_data.(v_name).x(t)<0
            vehicle_data.(v_name).x(t)=0;
        end
        if vehicle_data.(v_name).x(t)>800
            vehicle_data.(v_name).x(t)=800;
        end
        if vehicle_data.(v_name).y(t)<0
            vehicle_data.(v_name).y(t)=0;
        end
        if vehicle_data.(v_name).y(t)>800
            vehicle_data.(v_name).y(t)=800;
        end
        v_position{end+1}={v_name, vehicle_data.(v_name).x(t),vehicle_data.(v_name).y(t)};
    else
        %removing stopping vehicles
        v_sinfo= rmfield(v_sinfo,(v_name));
    end
end
end