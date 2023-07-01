
classdef Vehicle <handle
   properties
        % general params
        Ft_max, Fb_max, mass;
        Ft=0,Fb=0; Froll = 0;
        x=[0,0,0]',v=[0,0,0]',a=[0,0,0]';
        xa,xb,xc,xd;

        % angle
        w = 0; a_tan = 0; alpha = 0;
        thetf=0, thetw=0, L=4.7; W=2; WB = 2.7;
        Fsteer=0, Fsteer_max = 300, m_swhl = 30;

        % PID controllers
        refspd, safetail, lanectl, turner, crawl;
        ethresh = 0; ethresh_gain = 1; mode=0; counter = 100;

        math;
        dt = 1;

        % AI control systems
        recording = 0, tcnt = 1, mode_exp = []; mode_supd = []; mode_rec = []; Frec = [];
        % turn system
        Fsteer_actual = 0; Fcar = 0; B = 0; magv = 0;
        Fti=0; Fbi=0; thetwi=0; thetfi=0; magvi=0; Bi=0; Fcari=0;

   end
     
   methods
        %% INITIALIZATION
        function obj = Vehicle(Ft_max, Fb_max, mass)
            obj.math = smartmath();

            obj.Ft_max = Ft_max;
            obj.Fb_max = Fb_max;
            obj.mass = mass;
            obj.init_locn();

%             obj.refspd = PID(0.007,0.009,0.004,obj.dt, 5);
%             obj.refspd = PID(0.022, 0.002, 0.00, obj.dt, 5);
            obj.refspd = PID(0.00074325,  0.0037516, -0.0011668, obj.dt, 5);
            

%             obj.safetail = PID(1,0.1,0.01,obj.dt, 5);
            obj.safetail = PID(0.008, 0.001, 0.00, obj.dt, 5);
%             obj.safetail = PID(0.022, 0.002, 0.00, obj.dt, 5);
            obj.crawl = PID(0.022, 0.002, 0.00, obj.dt, 5);
            obj.lanectl = PID(0.001,0.0,0,obj.dt, 5);
            obj.turner = PID(0.001,0.001,0.0,obj.dt, 5);

       end
  
        %% display
        function show(obj, col)
            coors = [obj.x obj.xa obj.xb obj.xc obj.xd];
            plot( coors(1,:), coors(2,:), 'ob','MarkerSize',5,'MarkerFaceColor',col);
            % axis([-100 100 -100 100]);
            %             data = zeros(nsteps*3, 5);
            %                 data(t*3 : t*3+2,:) = coors;
            %                 disp(data(t*3 : t*3+2,:));
            %                 disp(obj.x);
            %                 disp(norm(obj.v));
        end
        function boxplot(obj)               % mark xa, xb, xc, xd points
            % rad_ians = -obj.thetf;
            rad_ians = -obj.thetf * (3.14159265 / 180);
        
	        obj.xa = [obj.x(1) + (obj.W / 2) * sin(rad_ians) + (obj.L / 2) * cos(rad_ians);
			        obj.x(2) + (obj.W / 2) * cos(rad_ians) - (obj.L / 2) * sin(rad_ians);
			        0];
        
	        obj.xb = [obj.x(1) + (-obj.W / 2) * sin(rad_ians) + (obj.L / 2) * cos(rad_ians);
			        obj.x(2) + (-obj.W / 2) * cos(rad_ians) - (obj.L / 2) * sin(rad_ians);
			        0];
        
	        obj.xc = [obj.x(1) + (obj.W / 2) * sin(rad_ians) - (obj.L / 2) * cos(rad_ians);
			        obj.x(2) + (obj.W / 2) * cos(rad_ians) + (obj.L / 2) * sin(rad_ians);
			        0];
        
	        obj.xd = [obj.x(1) + (-obj.W / 2) * sin(rad_ians) - (obj.L / 2) * cos(rad_ians);
			        obj.x(2) + (-obj.W / 2) * cos(rad_ians) + (obj.L / 2) * sin(rad_ians);
			        0];
                
        end
        function init_locn(obj)
            obj.x= [0 0 0]';
            obj.v = [1 0 0]'; obj.magv = 1;
            obj.Ft = 0; obj.Fb = 0;
            obj.a = [0 0 0]';
            obj.boxplot();

            obj.thetf = 0;
            obj.thetw = 0;
        end



        %% braking dist calc.
        function b = calc_brakedist(obj)            % calc brake dist of car
            b = 1.5 * obj.mass * norm(obj.v)^2 / obj.Fb_max;
        end

        function b = approx_brakedist_motorcycle(obj)
            b = 1.5 * 317 * norm(obj.v)^2 / 10000;
        end
        function b = approx_brakedist_truck(obj)
            b = 1.5 * 15000 * norm(obj.v)^2 / 60000;
        end
        function b = approx_brakedist_car(obj)
            b = 1.5 * 1200 * norm(obj.v)^2 / 20000;
        end
        
        %% angle & position calc.
        function bound_thetw(obj)
            if obj.thetw < -45
                obj.thetw = -45;
            elseif obj.thetw > 45
                obj.thetw = 45;
            end
        end
        function thetf_ = bound_ang(obj, thetf)
            thetf_ = thetf;
            if thetf >= 360
                thetf_ = thetf - 360;
            elseif thetf < 0
                thetf_ = thetf + 360;
            end
        end
        function update_xvthet(obj)
	        % update x, v, a
	        thet_Fnet = obj.thetf + obj.thetw;
            Fcar = obj.calc_Fcar();
	        obj.a = obj.math.ang2vect(Fcar / obj.mass, thet_Fnet);      % a = [_,_,_]'
            
            % BIKE MODEL FORMULAS
            B = atand(((obj.L/2)/obj.L) * tand(obj.thetw));         % B
            obj.a_tan = (Fcar / obj.mass) * cosd(obj.thetw - B);       % |a_tan |

            obj.x = obj.x + obj.dt * obj.v;                                                         % x = indegral(v)

            magv = norm(obj.v) + obj.dt * obj.a_tan;                                    % |v| = integral(a_tan)
            obj.v = [magv*cosd(obj.thetf + B)    magv*sind(obj.thetf + B)    0]';      % v = [_,_,_]' along (thetf + B)

            % set min velocity 0xa
            thetv = obj.math.vect2ang(obj.v);
            ang_btwn = obj.math.min_ang_btwn(thetv, obj.thetf + B);
            if ang_btwn > 90
                obj.v(1) = 0; obj.v(2) = 0;
            end

	        % final theta updates
            obj.thetf = obj.thetf + obj.dt * deg_((norm(obj.v) / obj.L) * cosd(B) * tand(obj.thetw));       % thetf = integral((v/L)*cos(B)*tan(thetw))
            obj.thetf = obj.bound_ang(obj.thetf);
            obj.thetw = obj.thetw + obj.dt^2 * 0.5 * obj.Fsteer / obj.m_swhl;
            obj.bound_thetw();
        end

        %% straight motion calc.
        function set_minv0(obj)     % ensure min velocity is 0
            thetv = obj.math.vect2ang(obj.v); 
            ang_btwn = obj.math.min_ang_btwn(thetv, obj.thetf);
            
            if ang_btwn > 90
                obj.v(1) = 0; obj.v(2) = 0;
            end
            
        end
        function Fcar = calc_Fcar(obj)          % calculate F car exerts
            obj.Froll = 0.497 * norm(obj.v)^2 + 4.0349 * norm(obj.v) + 97.33;
            Fcar = obj.Ft - obj.Fb - obj.Froll;
        end
        function Fsteer_actual = calc_Fsteer(obj)
            Fsteer_actual = obj.Fsteer;
        end

        %% pedals & steer ctrl
        function scaleup_Fpedals(obj, Fpid)             % scale up Fpedals
            obj.Fti = obj.Ft; obj.Fbi = obj.Fb;

            if Fpid > 0
	            obj.Ft = Fpid * obj.Ft_max;
	            obj.Fb = 0;
            else
	            obj.Fb = Fpid * -obj.Fb_max;
	            obj.Ft = 0;
            end
        end
        function [Ft_, Fb_] = scaleup_Fpedals2(obj, Fpid)       % scale up Ft, Fb
            if Fpid > 0
	            Ft_ = Fpid * obj.Ft_max;
	            Fb_ = 0;
            else
	            Fb_ = Fpid * -obj.Fb_max;
	            Ft_ = 0;
            end
        end    
        function scaleup_Fsteer(obj, Fpid)      % scale up Fsteer
            obj.Fsteer = Fpid * obj.Fsteer_max;
        end

        %% SINGLE timestep force cmds
        function steer(obj, thetd)
            Fpid = obj.turner.push(obj.thetf, thetd);
            obj.scaleup_Fpid_steer(Fpid);
        end
        function movestraight(obj, vset)
            Fpid = obj.refspd.push(norm(obj.v), vset);
            obj.scaleup_Fpedals(Fpid);
        end

   
        function update(obj)
            obj.update_xvthet(1, 1);
            obj.boxplot();
            obj.show("r");
        end

        %% braking dist calc. under diff. circumstances

        function F_max_fric = Ff_max(obj)
            F_max_fric  = 0.1 * obj.mass * 9.8;
        end
        function b=calc_brakedist_w_Ff(obj)
            b = 1.5 * obj.mass * norm(obj.v)^2 / obj.Ff_max();
        end
        function b=approx_brakedist_w_Ff_motorcycle(obj)
            b = 1.5 * 317 * norm(obj.v)^2 / obj.Ff_max();
        end
        function b=approx_brakedist_w_Ff_car(obj)
            b = 1.5 * 1200 * norm(obj.v)^2 / obj.Ff_max();
        end
        function b=approx_brakedist_w_Ff_truck(obj)
            b = 1.5 * 15000 * norm(obj.v)^2 / obj.Ff_max();
        end

        %% SIMULATIONS
        function run_generic_sim(obj, nsteps)
            obj.init_locn();
%             pause(1);
 
            obj.thetw = 0;
            vset = 8;
            obj.Fsteer = 10;
            for t = 1:nsteps
                obj.movestraight(vset);
                
%                 obj.followlane(1);

                obj.update();
%                 disp(norm(obj.v));
                disp([obj.thetw    obj.x(2)]);
%                 obj.update_xvthet();
%                 obj.boxplot();
%                 obj.show("r");
                if t > 20
%                     vset = 0;
%                     obj.Fsteer = -10;
                end
                if t > 35
%                     vset = 3;
                end

                axis([-300 300 -300 300]);
                pause(0.1);

            end

        end

        %% cruise control helpers

        function choose_Fmode(obj, Ft_ref, Fb_ref, Ft_safe, Fb_safe, Ft_crawl, Fb_crawl)
            if obj.mode == 0
                obj.Ft = Ft_ref; obj.Fb = Fb_ref;
            elseif obj.mode == 1
                obj.Ft = Ft_safe; obj.Fb = Fb_safe;
            elseif obj.mode == 2
                obj.Ft = Ft_crawl; obj.Fb = Fb_crawl;
            end
        end

        function roadfric_constrain(obj)            % constrain Ft, Fb for icy roads
            if obj.calc_Fcar() > 0.1 * (9.8 * obj.mass)
                obj.Ft = 0.1 * obj.mass * 9.8 + obj.Froll;
            elseif obj.calc_Fcar() < -0.1 * (9.8 * obj.mass)
                obj.Fb = 0.1 * (9.8 * obj.mass) - obj.Froll;
            end
        end

        function historesis(obj, want_mode)         % smoothen/stall mode transitions
            if want_mode == 0 && obj.mode == 1
                if obj.counter == 100
                    obj.counter = 1;
                else
                    obj.counter = obj.counter + 1;
                end

                if obj.counter > 10 && obj.counter ~= 100
                    obj.counter = 100;
                    obj.mode = want_mode;
                end
            elseif obj.mode == 0 && want_mode == 2
                obj.mode = 1;
            else
                obj.mode = want_mode;
            end
        end

        %% CRUISE CONTROL DRIVER FUNCTIONS
        function runCruiseCtrl(obj, n0, vset0, vset1, ntimesteps)
	        obj.ethresh = 17;
            obj.x = [-40 0 0]'; obj.v = [0 0 0]';

            for t = 1:ntimesteps
                obj.cruise_ctl(n0, vset0, vset1);

                obj.update_xvthet();
                n0.update_xvthet();
                obj.boxplot();
                n0.boxplot();

                % display cars
                obj.show("r");
                hold on;
                n0.show("b");
                hold off;
                axis([-100 300 -100 100]);
                pause(0.1);

                %------- change vset(lead) -------
                %------- detect collision -------
                if obj.detect_collision(n0) == 1
                    break
                end

            end
        end


        function res = detect_collision(obj, n0)
            res = 0;
            if (obj.mode < 0)
                disp([" WARNING " obj.mode]);
            end
            if obj.x(1) > n0.x(1) - (obj.L/2 + n0.L/2)
                disp("COLLISION!!!\n");
                res = 1;
            end
        end

        function cruise_ctl(obj, n0, vset0, vset1)      % normal cruise ctl

            %------- NODE 1 -------
            followdist = norm(obj.x - n0.x) - obj.L/2 - n0.L/2;

            dsafe = obj.calc_brakedist() - n0.approx_brakedist_motorcycle();% - n0.calc_brakedist();
            obj.ethresh = (1.3 * norm(obj.v));

            % historesis
            want_mode = (followdist - dsafe) <0;
            if norm(obj.v) < 2 && norm(n0.v) < 2 && followdist < 8
                want_mode = 2;
            end
            obj.historesis(want_mode);

	        % run both PID controllers
	        [Ft_ref, Fb_ref] = obj.scaleup_Fpedals2(obj.refspd.push(norm(obj.v), vset1));	% calc Ft_ref, Fb_ref
	        [Ft_safe, Fb_safe] = obj.scaleup_Fpedals2(obj.safetail.push(0,followdist - dsafe - obj.ethresh));		% calc Ft_safe, Fb_safe
            [Ft_crawl, Fb_crawl] = obj.scaleup_Fpedals2(obj.crawl.push(0,followdist - dsafe - 5));     % calc Ft_crawl, Fb_crawl

            % generate value for Fcar, based on cruise control mode
	        obj.choose_Fmode(Ft_ref, Fb_ref, Ft_safe, Fb_safe, Ft_crawl, Fb_crawl);		% choose Ft, Fb based on mode & ethresh

%             disp([ followdist norm(obj.v) obj.Ft obj.Fb obj.mode ]);

            %------- NODE 0 -------
            n0.scaleup_Fpedals(n0.refspd.push(norm(n0.v), vset0));

        end
   
        function cruise_ctl2(obj, n0, vset0, vset1)     % icy road cruise ctl
            
            %------- NODE 1 -------
            followdist = norm(obj.x - n0.x) - obj.L/2 - n0.L/2;

            dsafe = obj.calc_brakedist_w_Ff() - n0.approx_brakedist_w_Ff_car();% - n0.calc_brakedist();
            obj.ethresh = obj.ethresh_gain * (3.7 * max(norm(obj.v) - norm(n0.v), 0) + 1.3 * norm(obj.v) + 5);

            % historesis
            want_mode = (followdist - dsafe - obj.ethresh)<0;
            if norm(obj.v) < 2 && norm(n0.v) < 2 && followdist < 8
                want_mode = 2;
            end
            obj.historesis(want_mode);

	        % run both PID controllers
	        [Ft_ref, Fb_ref] = obj.scaleup_Fpedals2(obj.refspd.push(norm(obj.v), vset1));	% calc Ft_ref, Fb_ref
	        [Ft_safe, Fb_safe] = obj.scaleup_Fpedals2(obj.safetail.push(0,followdist - dsafe - obj.ethresh));		% calc Ft_safe, Fb_safe
            [Ft_crawl, Fb_crawl] = obj.scaleup_Fpedals2(obj.crawl.push(0,followdist - dsafe - 3));     % calc Ft_crawl, Fb_crawl

            % generate value for Fcar, based on cruise control mode
	        obj.choose_Fmode(Ft_ref, Fb_ref, Ft_safe, Fb_safe, Ft_crawl, Fb_crawl);		% choose Ft, Fb based on mode & ethresh
            obj.roadfric_constrain();

            disp([ followdist norm(obj.v) obj.Ft obj.Fb obj.mode ]);

            %------- NODE 0 -------
            n0.scaleup_Fpedals(n0.refspd.push(norm(n0.v), vset0));
            n0.roadfric_constrain();

   
        end

        %%
        function autotune_refspd(obj)
            obj.refspd.set_ks([ 0.0 0 0]);

            t_ = 1:1:40;
            cases = [];
            b = -3; % v0 = 0.084;
            for vset = 10:1:18
                dt = ((vset)/0.363)^0.5;
                m = 3.193/dt;
                vexp = 1 ./(1 + 2.7128.^-(m.*t_ + b));
                vset_ = t_>1;
                Fexp = m.*(1-(m.*t_+b));
                cases(end+1:end+3,:) = [vset_ * vset; vexp * vset; Fexp * vset];
            end

            data = zeros(size(cases,1),numel(t_));
            ne = 18;
            for e = 1:ne
                for i = 1:3:size(cases,1)
                    obj.v = [2*cases(i+1,1) 0 0]'; obj.refspd.reset();
                    for t = 1:size(cases,2)
                        vset = cases(i,t);
                        vexp = cases(i+1, t);
                        Fexp = cases(i+2, t);
                        obj.movestraight(vset);
%                         disp(obj.Ft);
%                         disp(obj.Fb);
                        obj.update_xvthet();

                        if e == ne
                            data(i,t) = vexp;
                            data(i+1,t) = obj.v(1);
                        end

                        C = Fexp - obj.Ft - obj.Fb;
                        if C > 50
                            break;
                        end
                        obj.refspd.kp = obj.refspd.kp - (0.00007  / 27000 / size(cases,1)) * C * obj.refspd.errors(end);
                        obj.refspd.ki = obj.refspd.ki - (0.00007 / 27000 / size(cases,1)) * C * obj.refspd.integ;
                        obj.refspd.kd = obj.refspd.kd - (0.00007 / 27000) * C * (obj.refspd.errors(end) - obj.refspd.errors(end-2));
                    end
                end
            end

            for row = 1:size(data,1)
                plot(data(row,:));
                hold on;
            end
            hold off;
            disp([obj.refspd.kp obj.refspd.ki obj.refspd.kd]);
            axis([0 100 0 100]);
        end



        %% CRUISE CONTROL DRIVER FUNCTIONS
        function runCruiseCtrl_(obj, n0, vset0, vset1, ntimesteps)
	        obj.ethresh = 10;
            obj.x = [-40 0 0]';
            
            for t = 1:ntimesteps
                obj.cruise_ctl(n0, vset0, vset1);

                obj.update_xvthet4(1, 1);
                obj.boxplot();

                n0.update_xvthet4(1, 1);
                n0.boxplot();

                % display cars
                obj.show("r");
                hold on;
                n0.show("b");
                hold off;
                axis([-100 2000 -100 100]);
                pause(0.1);

                %------- change vset(lead) -------
                if (t > 15)
                    vset0 = 0;
                end
%                 if (t > 40)
%                     vset0 = 0;
%                 end

                %------- detect collision -------
                if (obj.mode < 0)
                    disp([" WARNING " obj.mode]);
                end
                if obj.x(1) > n0.x(1) - 4.7
                    disp("COLLISION!!!\n");
                    break
                end

            end
        end


        function cruise_ctl_(obj, n0, vset0, vset1)      % normal cruise ctl

            %------- NODE 1 -------
            followdist = norm(obj.x - n0.x) - obj.L/2 - n0.L/2;

            dsafe = obj.calc_brakedist() - n0.approx_brakedist_car();% - n0.calc_brakedist();
            obj.ethresh = obj.ethresh_gain * (3.7 * max(norm(obj.v) - norm(n0.v), 0) + 1.3 * norm(obj.v) + 5);

%             % FIXME - historesis
%             if obj.mode < 0 && (followdist - dsafe - obj.ethresh > 0) && (obj.counter <  5)
%                 obj.counter = 0;%obj.counter + 1;
%             elseif obj.mode < 0 && (followdist - dsafe - obj.ethresh > 0) && (obj.counter >= 5)
%                 obj.counter = 0;
%                 
%             end

            
            if norm(obj.v) < 2 && norm(n0.v) < 2
                obj.ethresh = 5;
            end
            obj.mode = followdist - dsafe - obj.ethresh;

	        % run both PID controllers
	        [Ft_ref, Fb_ref] = obj.scaleup_Fpedals2(obj.refspd.push(norm(obj.v), vset1));	% calc Ft_ref, Fb_ref
	        [Ft_safe, Fb_safe] = obj.scaleup_Fpedals2(obj.safetail.push(0,obj.mode));		% calc Ft_safe, Fb_safe
            [Ft_crawl, Fb_crawl] = obj.scaleup_Fpedals2(obj.crawl.push(0,followdist - dsafe - 5));     % calc Ft_crawl, Fb_crawl

            % generate value for Fcar, based on cruise control mode
	        obj.choose_Fmode(Ft_ref, Fb_ref, Ft_safe, Fb_safe, Ft_crawl, Fb_crawl);		% choose Ft, Fb based on mode & ethresh

            disp([ obj.ethresh followdist norm(obj.v) obj.Ft obj.Fb obj.ethresh_gain obj.mode ]);

            %------- NODE 0 -------
            n0.scaleup_Fpedals(n0.refspd.push(norm(n0.v), vset0));

        end
   
                        

   end
end



