
function y_out = Flocking_Matlab()

close all;
set(0,'Defaultlinelinewidth',5, 'DefaultlineMarkerSize',6,...
    'DefaultTextFontSize',5, 'DefaultAxesFontSize',18);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% initialize parameters
N = 100;         %No. of boids
frames = 50;    %No. of frames in movie
limit = 100;       %Axis limits
L=limit*2; 
P = 10;          %Spread of initial position (gaussian)
V = 10;          %Spread of initial velocity (gaussian)
delta = 1;       %Time step
c1 = .00001;    %Attraction scaling factor (.00001)
c2 = .01;       %Repulsion scaling factor (.01)
c3 = 1;          %Heading scaling factor (1)
c4 = .1;        %Randomness scaling factor (.1)
vlimit = 1;      %Maximum velocity


%Initialize
p = P*randn(2,N);
v = V*randn(2,N);
v = v./vecnorm(v);
figure(); 

%Main loop
for k=1:frames
    v1=zeros(2,N);
    v2=zeros(2,N);
    v4=zeros(2,N);

    %Compute heading [alignment] v3
    v3 = [sum(v(1 ,:))/N; sum(v(2 ,:))/N]*c3;
    %Limit max velocity
    if(vecnorm(v3) > vlimit), v3 = v3*vlimit/vecnorm(v3); end
    for n=1:N
        for m=1:N
            if m ~= n
				r = p(:, m) - p(:, n);
                
                if r(1)>L/2, r(1) = r(1)-L;
                elseif r(1)<-L/2, r(1) = r(1)+L;
                end
                
                if r(2)>L/2, r(2) = r(2)-L;
                elseif r(2)<-L/2, r(2) = r(2)+L;
                end
                
                %Compute distance between agents rmag
                rmag = sqrt(r(1)^2 + r(2)^2);
                %Compute Attraction v1
                v1(: ,n) = v1(: ,n) + c1*r;
                %Compute repulsion (non-linear scaling)
                v2(: ,n) = v2(: ,n) - c2*r/(rmag^2);

            end
        end

        %Compute random velocity component v4
        v4(: ,n) = c4*randn(2,1);
        %Update velocity and position
        v(: ,n) = v(: ,n) + v1(: ,n) + v2(: ,n) + v3 + v4(: ,n);
    end

    %Update position
    p = p + v * delta;

    % Periodic boundary 
    tmp_p = p;
    
    tmp_p(1,p(1,:)>L/2) = tmp_p(1,p(1,:)>L/2) - L;
    tmp_p(2,p(2,:)>L/2) = tmp_p(2,p(2,:)>L/2) - L;
    tmp_p(1,p(1,:)<-L/2)  = tmp_p(1,p(1,:)<-L/2) + L;
    tmp_p(2,p(2,:)<-L/2)  = tmp_p(2,p(2,:)<-L/2) + L;
    
    p = tmp_p;
    
    %Update plot:
    plot(p(1,:),p(2,:),'k.','Markersize',10); hold on; 
    quiver(p(1,:),p(2,:),v(1,:),v(2,:)); %For drawing velocity arrows
    axis([-limit limit -limit limit]); axis square; 
    drawnow; 
    hold off;
    pause(0.1);
end
y_out=0; 


end




