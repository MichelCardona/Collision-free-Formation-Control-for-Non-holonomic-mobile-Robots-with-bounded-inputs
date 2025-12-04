classdef TurtleBot3_WafflePI < handle
    properties (Access = private)
        bot_n = [];
        bot = [];

        T = [];
    end
    
    methods
        function obj = TurtleBot3_WafflePI(bot_ip,T)
            try
                obj.bot_n = bot_ip(21);
                disp(['Connecting to TurtleBot3 Waffle PI #' num2str(obj.bot_n) '...'])
                
                if(nargin>1)
                    obj.T = T;
                else
                    obj.T = eye(3,3);
                end

                obj.bot = turtlebot(bot_ip,11311);

                % Lidar
                obj.bot.LaserScan.TopicName = ['/bot' obj.bot_n '/scan'];
                
                % Odometry
                obj.bot.Odometry.TopicName = ['/bot' obj.bot_n '/odom'];
                obj.bot.OdometryReset.TopicName = ['/bot' obj.bot_n '/reset'];

                % Velocity Command
                obj.bot.Velocity.TopicName = ['/bot' obj.bot_n '/cmd_vel'];

                disp(' done.')
            catch E
                error(E)
            end
        end

        %% Velocity Command
        function Set_Velocity (obj,v,w)
            try
                v = max(v,-0.2); v = min(v,0.2);
                w = max(w,-0.9); w = min(w,0.9);

                setVelocity(obj.bot,v,w)
            catch E
                error(E)
            end
        end

        %% Odometry
        function Reset_Odometry (obj)
            try
                disp(['TurtleBot3 #' num2str(obj.bot_n) ' restarting odometry...'])
                resetOdometry(obj.bot)
                pause(2)

                setVelocity(obj.bot,0,0)
                setVelocity(obj.bot,0,0)
                setVelocity(obj.bot,0,0)

                disp(' done.')
            catch E
                error(E)
            end
        end

        function p = Get_Pose (obj)
            try
                odom = getOdometry(obj.bot);
                x = odom.Position(1);
                y = odom.Position(2);
                theta = odom.Orientation(1);

                P = obj.T*[cos(theta) -sin(theta) x; sin(theta) cos(theta) y; 0 0 1];
                p = [P(1,3); P(2,3); atan2(P(2,1),P(1,1))];
            catch E
                error(E)
            end
        end

        %% Lidar
        function [Angles,Ranges] = Get_LaserScans (obj)
            d_min = 0.01;
            d_max = 3.0;
            
            try
                [scan,~] = getLaserScan(obj.bot);

                Angles = scan.Angles;
                Ranges = scan.Ranges;

                I = d_min<=Ranges & Ranges<=d_max;

                Angles = Angles(I);
                Ranges = Ranges(I);
            catch E
                error(E)
            end
        end
    end
end
