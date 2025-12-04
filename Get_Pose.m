%%
function p = Get_Pose (robot_Sub)
    robot_Msg = receive(robot_Sub);
    
    Point = robot_Msg.Pose.Position;
    t = [Point.X Point.Y Point.Z]';

    Q = robot_Msg.Pose.Orientation;
    R = quat2rotm([Q.W Q.X Q.Y Q.Z]);

    Tbc = [R t; 0 0 0 1];
    Tab = [0 0 1 0; 1 0 0 0; 0 1 0 0; 0 0 0 1];
    Tac = Tab*Tbc;

    angles = rotm2eul(Tac(1:3,1:3));

    x = Tac(1,4);
    y = Tac(2,4);
    theta = angles(1);
    theta = atan2(sin(theta),cos(theta));

    p = [x y theta]';
end
