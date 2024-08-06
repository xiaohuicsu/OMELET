function [v_sinfo,MF] = plan_mig(v_sinfo,G,link_rate,len_time,lamda)
mig_count=0;
MF=0;
Mig_set={};
v_id_set=fieldnames(v_sinfo);
for k=1:length(v_id_set)
    v_iid=v_id_set{k};
    if v_sinfo.(v_iid).s_place~=v_sinfo.(v_iid).s_mig
        mig_count=mig_count+1;
        Mig_set(mig_count,:)={v_iid,v_sinfo.(v_iid).s_size,v_sinfo.(v_iid).s_place,v_sinfo.(v_iid).s_mig};
    end
end
if mig_count>0
    Mig_set = sortrows(Mig_set,2);% ascend service according to the s_size
end

for k=1:mig_count
    success=0;
    [nodepaths,edgepaths] = allpaths(G,Mig_set{k,3},Mig_set{k,4});
    path_rate={};
    % acesend path according to the rate, Best-fit
    for n=1:length(edgepaths)
        selectpath=edgepaths{n,1};
        path_rate(n,1)={selectpath};%path info
        path_rate(n,2)={min(link_rate(selectpath))};%path rate
    end
    path_rate = sortrows(path_rate,2);
    
    for m=1:length(edgepaths)
        hop_count=length(path_rate{m,1});
        min_rate=Mig_set{k,2}/(len_time-lamda*hop_count);
        if path_rate{m,2}>=min_rate
            v_sinfo.(Mig_set{k,1}).s_route={path_rate{m,1},min_rate};
            link_rate(path_rate{m,1})=link_rate(path_rate{m,1})-min_rate;
            success=1;
            break
        end
    end
    if success==0
        v_sinfo.(Mig_set{k,1}).s_mig=v_sinfo.(Mig_set{k,1}).s_place;
        MF=MF+1;
    end

end