% x = X2(end,:);
% y = Y2(end,:);
% theta = theta2(end,:)
% u = cos(theta);
% v = sin(theta);
% figure
% q = quiver(x,y ,u,v)
% q.ShowArrowHead = 'off'


% figure
% plot(X2(120,:))
% hold on
% for i = 1:20
%     plot(X2(120-i,:))
%     hold on
% end
%legend('it. 120','it. 119','it. 118','it. 117','it.116','it. 115','it. 114','it.113','it.112','it.111','it.110')

% plot(xo_his_linear(2,:))
% hold on
% plot(dataAICOlinear(2,:))
% hold on
% plot(datasmootherlinear(2,:))
% legend('optimal control','AICO', 'Kalman Filter- RTS smoother')



% 
 h = 0.001;
 sz = size(dataAICOunicyclewithdisturbance_2000);
% %  sz1 = size(dataAICOunicycledisturbance1);
for n = 1:100
    X2(n,:) = dataAICOunicyclewithdisturbance_2000((3*n -2),:);
    Y2(n,:) =dataAICOunicyclewithdisturbance_2000(3*n -1,:);
    theta2(n,:) = dataAICOunicyclewithdisturbance_2000(3*n ,:);
%     
%     X2(n,:) = dataAICOunicyclewithdisturbance21((3*n -2),:);
%     Y2(n,:) = dataAICOunicyclewithdisturbance21(3*n -1,:);
%     theta2(n,:) = dataAICOunicyclewithdisturbance21(3*n ,:);
end





% theta = dataAICOunicyclewithdisturbance1(3,:);
theta = theta2(end,:);
for i = 2:sz(2)
    vx(i-1) = (X2(end,i)-X2(end,i-1))/h;
    vy(i-1) = (Y2(end,i)-Y2(end,i-1))/h;
    v(i-1)  = sqrt(vx(i-1)^2 + vy(i-1)^2);
    vdir(i-1) = atan(vy(i-1)/vx(i-1));
end
t = [0:h:2.0];
vel_set = 10*heaviside(t);
figure
plot(vx)
hold on
plot(vy)
hold on 
plot(v)
hold on
plot(vel_set)
ylabel('v')
xlabel('timestep')
legend('$v_x$', '$v_y$', '$v_{norm} = \sqrt{v_x^{2} + v_y^2}$', '$v_{set}=10$', 'Interpreter', 'latex')
figure
plot(theta(2:end))
hold on
plot(vdir)
legend('$\theta$','$vel_{dir}$','interpreter', 'latex')



