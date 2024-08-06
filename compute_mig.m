function migration_cost = compute_mig(v_sinfo)
unit_cost=0.01;
migration_cost=0;
count=0;
v_id_set=fieldnames(v_sinfo);
for k=1:length(v_id_set)
    v_id=v_id_set{k};
    if v_sinfo.(v_id).s_place~=v_sinfo.(v_id).s_mig
        count=count+1;
        migration_cost=migration_cost+unit_cost*v_sinfo.(v_id).s_size;
    end
end
migration_cost=migration_cost/length(v_id_set);%avg_sum



