
clc;
clear;
close all;

%Build the graph---------------------
unit_cost=0.1;

%the pair of nodes of each edge
s1 = [1 2 4 5 7 8 2 5];
s2 = [2 3 5 6 8 9 5 8];
%edge information
link_cost = unit_cost*ones(1,length(s1));

%node
G = graph(s1,s2,link_cost);
plot(G);
num_vehicle=200;

P_s=4;
W_s=30+(60-30)*rand(1,length(s1));
unit_rate=W_s.*log2(1+P_s);


size_set=400;

%parameter for Gibbs
omega_set = 10^(-5);

%parameter for Lyapunov
V_set=0.09;

num_iter=2000;
MF=0.001;
num_time=150;% time slots
len_time=30;%

Avg_que=zeros(1,length(size_set));%Objection value of queue
Avg_SLAV=zeros(1,length(size_set));
Avg_mig=zeros(1,length(size_set));%Objection value of migration
Avg_sum=zeros(1,length(size_set));
Gibbs=zeros(num_iter,length(size_set));

Obj_queLen=zeros(length(size_set),num_time+1);
Obj_SLAV=zeros(length(size_set),num_time+1);
Obj_queVal=zeros(length(size_set),num_time+1);%Objection value of queue
Obj_mig=zeros(length(size_set),num_time);%Objection value of migration
Obj_sum=zeros(length(size_set),num_time);

phi_max=0.1;% maximum average SLA violation rate
lamda=0.01;% latency of one hop


axis_val=[0 400 800];
sbs_axis={};
for i=1:length(axis_val)
    for j=1:length(axis_val)
        sbs_axis=[sbs_axis [axis_val(j),axis_val(i)]];
    end
end
num_sbs=length(sbs_axis);

%Initiliaze vehilce trace for each time slot-------------------------------
vehicle_data = struct;

for i = 1:num_vehicle
    v_name=['v',num2str(i)];
    data_name=sprintf('vehicle_data%d.mat',i);
    vehicle_data.(v_name)=load(data_name);
end
qos_exp = 0.45+(0.9-0.45)*rand(1,num_vehicle);
v_t_size=ones(1,num_vehicle)*200;%service request size (mb)

s_place=randi(num_sbs,1,num_vehicle);%initial service placement
s_mig=s_place;%initial service migration

s_route=cell(num_vehicle,2);%row: route and min_rate

for size_index=1:length(size_set)
    s_size = size_set(size_index) + (2*size_set(size_index)-size_set(size_index))*rand(1,num_vehicle);

    for oga_index=1:length(omega_set)
        for V_value=1:length(V_set)
            %Initialize the vehicle service information--------------------------------
            v_sinfo=struct;
            for i = 1:num_vehicle
                v_name=['v',num2str(i)];
                v_sinfo.(v_name).v_size=v_t_size(i);
                v_sinfo.(v_name).qos=qos_exp(i);
                v_sinfo.(v_name).s_size=s_size(i);
                v_sinfo.(v_name).s_place=s_place(i);
                v_sinfo.(v_name).s_mig=s_mig(i);
                v_sinfo.(v_name).s_route=s_route(i,:);
            end
            %--------------------------------------------------------------------------

            queue_len=0;
            val_que=0;
            val_mig=0;
            val_sum=0;
            for t=1:num_time %time slot
                if t==1
                    [v_Axis,v_sinfo]=getVP(vehicle_data,v_sinfo,t);

                    %Initialization with no migration
                    phi_current=compute_phi(v_Axis,sbs_axis,v_sinfo);%(v_pos,s_information)
                    queue_len=max((phi_current-phi_max),0);
                    Obj_SLAV(size_index,t)=phi_current;
                    Obj_queLen(size_index,t)=queue_len;
                end

                [v_nextAxis,v_sinfo]=getVP(vehicle_data,v_sinfo,t*len_time+1);
                phi_next=compute_phi(v_nextAxis,sbs_axis,v_sinfo);

                val_que = queue_len*(phi_next-phi_max);
                val_mig = 0;
                val_sum = val_que;


                for iter=1:num_iter
                    v_index = randi(length(v_nextAxis));%random index for choosing vehicle
                    v_id=v_nextAxis{v_index}{1};

                    v_id_nAxis=[v_nextAxis{v_index}{2},v_nextAxis{v_index}{3}];
                    v_size=v_sinfo.(v_id).v_size;
                    v_qos=v_sinfo.(v_id).qos;
                    v_s_size=v_sinfo.(v_id).s_size;
                    v_s_place=v_sinfo.(v_id).s_place;
                    v_s_mig=v_sinfo.(v_id).s_mig;
                    v_s_route=v_sinfo.(v_id).s_route;

                    ava_sbs = findAvaSBS(v_id_nAxis,sbs_axis,v_size,v_qos);%elements in array: sbs index
                    ava_sbs=[ava_sbs,v_s_place];
                    sbs_index = ava_sbs(randi(length(ava_sbs)));%random migration destination SBS

                    if sbs_index~=v_s_mig
                        v_sinfo_temp=v_sinfo;
                        v_sinfo_temp.(v_id).s_mig=sbs_index;
                        %Greedy migration path plannning---------------------------
                        link_rate = unit_rate*6;
                        [v_sinfo_temp,MF] = plan_mig(v_sinfo_temp,G,link_rate,len_time,lamda);
                        %calculate phi(t+1) with potential migration
                        phi_next_temp=compute_phi(v_nextAxis,sbs_axis,v_sinfo_temp);
                        %Calculate objection value with potential migration
                        val_que_temp = queue_len*(phi_next_temp-phi_max);
                        %Calculate objection value of migration
                        val_mig_temp=compute_mig(v_sinfo_temp);

                        val_sum_temp = val_que_temp + V_set(V_value)*val_mig_temp+MF*0.001;
                        p_bias = val_sum_temp-val_sum;
                        p=1/(1+exp(p_bias/omega_set(oga_index)));
                        rand_num=rand();
                        if rand_num()<=p
                            v_sinfo=v_sinfo_temp;
                            phi_next=phi_next_temp;
                            val_que=val_que_temp;
                            val_mig=val_mig_temp;
                            val_sum=val_sum_temp;
                        end
                    end
                end
                Obj_queVal(size_index,t) = val_que;
                Obj_mig(size_index,t) = val_mig;
                Obj_sum(size_index,t) = val_sum;
                queue_len=max(queue_len+phi_next-phi_max,0);
                Obj_SLAV(size_index,t+1)=phi_next;
                Obj_queLen(size_index,t+1)=queue_len;

                v_id_set=fieldnames(v_sinfo);
                for k=1:length(v_id_set)
                    v_iid=v_id_set{k};
                    v_sinfo.(v_iid).s_place=v_sinfo.(v_iid).s_mig;
                end
            end
            Avg_que(size_index)=sum(Obj_queLen(size_index,1:num_time+1))/num_time;
            Avg_SLAV(size_index)=sum(Obj_SLAV(size_index,1:num_time+1))/num_time;
            Avg_mig(size_index)=sum(Obj_mig(size_index,:))/num_time*num_vehicle;
            Avg_sum(size_index)=sum(Obj_sum(size_index,:))/num_time;
        end
    end
end
