
classdef Carr <handle
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
        function obj = Carr(Ft_max, Fb_max, mass)
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

        function cases_ = make_cases(obj, ntsteps)
            t_ = 1:1:ntsteps;
            ncases = 1;
            cases_ = zeros(2*ncases, ntsteps);
            amp = 3.4;
            for i = 1:ncases
                cases_(i,:) = t_;
                cases_(i+1,:) = -0.3 + amp* 1 ./ (1 + 2.7128.^((-0.3) * (t_ -5)));
            end
        end


        % SCRAP THIS
        function update_xvthet2(obj)
            obj.magvi = obj.magv; obj.thetfi = obj.thetf; obj.Bi = obj.B; obj.thetwi = obj.thetw;

            % 1
            obj.x = obj.x + obj.dt * obj.v;     % x = indegral(v)
            % 2
            obj.v = [obj.magv*cosd(obj.thetf + obj.B)    obj.magv*sind(obj.thetf + obj.B)    0]';      % v = [_,_,_]' along (thetf + B)

            % 3
            obj.thetf = obj.thetf + obj.dt * deg_((norm(obj.v) / obj.L) * cosd(obj.B) * tand(obj.thetw));       % thetf = integral((v/L)*cos(B)*tan(thetw))
            obj.thetf = obj.bound_ang(obj.thetf);
            
            % 4
            obj.magv = norm(obj.v) + obj.dt * obj.a_tan;                                    % |v| = integral(a_tan)
            % 5
            obj.a_tan = (obj.Fcar / obj.mass) * cosd(obj.thetw - obj.B);       % |a_tan |
            % 6
	        obj.a = obj.math.ang2vect(obj.Fcar / obj.mass, obj.thetf + obj.thetw);      % a = [_,_,_]'
            % 7
            obj.Fcar = obj.calc_Fcar();
            
            % 8
            obj.B = atand(((obj.L/2)/obj.L) * tand(obj.thetw));         % B
            % 9
            obj.thetw = obj.thetw + obj.dt^2 * 0.5 * obj.Fsteer_actual / obj.m_swhl;
            obj.bound_thetw();
            % 10
            obj.Fsteer_actual = obj.calc_Fsteer();

        end

        % SCRAP THIS
        function update_xvthet3(obj)
            obj.magvi = obj.magv; obj.thetfi = obj.thetf; obj.Bi = obj.B; obj.thetwi = obj.thetw;

	        % update x, v, a
	        thet_Fnet = obj.thetf + obj.thetw;
            obj.Fcar = obj.calc_Fcar();
	        obj.a = obj.math.ang2vect(obj.Fcar / obj.mass, thet_Fnet);      % a = [_,_,_]'
            
            % BIKE MODEL FORMULAS
            obj.B = atand(((obj.L/2)/obj.L) * tand(obj.thetw));         % B
            obj.a_tan = (obj.Fcar / obj.mass) * cosd(obj.thetw - obj.B);       % |a_tan |

            obj.magv = norm(obj.v) + obj.dt * obj.a_tan;                                    % |v| = integral(a_tan)
            obj.v = [obj.magv*cosd(obj.thetf + obj.B)    obj.magv*sind(obj.thetf + obj.B)    0]';      % v = [_,_,_]' along (thetf + B)

            % set min velocity 0xa
            thetv = obj.math.vect2ang(obj.v);
            ang_btwn = obj.math.min_ang_btwn(thetv, obj.thetf + obj.B);
            if ang_btwn > 90
                obj.v(1) = 0; obj.v(2) = 0;
            end

            obj.x = obj.x + obj.dt * obj.v;                                                         % x = indegral(v)

	        % final theta updates
            obj.thetf = obj.thetf + obj.dt * deg_((norm(obj.v) / obj.L) * cosd(obj.B) * tand(obj.thetw));       % thetf = integral((v/L)*cos(B)*tan(thetw))
            obj.thetf = obj.bound_ang(obj.thetf);
            obj.thetw = obj.thetw + obj.dt^2 * 0.5 * obj.Fsteer / obj.m_swhl;
            obj.bound_thetw();
        end

        function dCy_dFsteer = update_xvthet4(obj, supx, supy)

	        % a = Fcar/m = (Ft - Fb - Froll)/m
	        thet_Fnet = obj.thetf + obj.thetw;
            obj.Fcar = obj.calc_Fcar();
	        obj.a = obj.math.ang2vect(obj.Fcar / obj.mass, thet_Fnet);      % a = [_,_,_]'
            
            % B, |a_tan| (calculated from thetw, B)
            obj.B = atand(((obj.L/2)/obj.L) * tand(obj.thetw));         % B
            obj.a_tan = (obj.Fcar / obj.mass) * cosd(obj.thetw - obj.B);       % |a_tan |

%             obj.a_tan = [obj.a_tan *cosd(obj.thetf + obj.B) ;
%                 obj.a_tan *sind(obj.thetf + obj.B);
%                 0];

            % magv, v
            obj.magv = norm(obj.v) + obj.dt * obj.a_tan;                                    % |v| = integral(a_tan)
            obj.v = [obj.magv*cosd(obj.thetf + obj.B)    obj.magv*sind(obj.thetf + obj.B)    0]';      % v = [_,_,_]' along (thetf + B)
%             obj.v = obj.v + obj.a_tan * obj.dt;

            % set min velocity 0
            thetv = obj.math.vect2ang(obj.v);
            ang_btwn = obj.math.min_ang_btwn(thetv, obj.thetf + obj.B);
            if ang_btwn > 90
                obj.v(1) = 0; obj.v(2) = 0;
            end

            % x = [_,_,_]'
            obj.x = obj.x + obj.dt * obj.v;                                                         % x = indegral(v)




            datan_dthetw = (obj.Fcar / obj.mass) * (-1)*sind(obj.thetw - obj.B);
            datan_dB = (obj.Fcar / obj.mass) * sind(obj.thetw - obj.B);
            dB_dthetw = ( 0.5*(secd(obj.thetw))^2 / (1 + 0.25*(tand(obj.thetw))^2) );
            dthetw_dFsteer = deg_(0.5*obj.dt^2);
            dvy_dmagv = sind(obj.thetf + obj.B);
            dmagv_datan = obj.dt;
            dvy_dB = obj.magv*cosd(obj.thetf + obj.B);

%             dvy_dthetf = obj.magv*cosd(obj.thetf + obj.B);

            Cx = obj.x(1) - supx; Cy = obj.x(2) - supy;
            c1 = Cy * obj.dt * dvy_dmagv * dmagv_datan * datan_dthetw * dthetw_dFsteer;
            c2 = Cy * obj.dt * dvy_dmagv * dmagv_datan * datan_dB * dB_dthetw * dthetw_dFsteer;
            c3 = Cy * obj.dt * dvy_dB * dB_dthetw * dthetw_dFsteer;

            dvy_dthetf = obj.magv*cosd(obj.thetf + obj.B);
            dthetf_dmagvi = deg_((1 / obj.L) * cosd(obj.Bi) * tand(obj.thetwi));
            dthetf_dBi = deg_((obj.magvi / obj.L) * (-1)*sind(obj.Bi) * tand(obj.thetwi));
            dthetf_dthetwi = deg_((obj.magvi / obj.L) * cosd(obj.Bi) * (secd(obj.thetwi))^2);
            dthetwi_dFsteeri = deg_(0.5*obj.dt^2);
            dBi_dthetwi = ( 0.5*(secd(obj.thetwi))^2 / (1 + 0.25*(tand(obj.thetwi))^2) );
            dmagvi_datani = obj.dt;
            datani_dthetwi = (obj.Fcari / obj.mass) * (-1)*sind(obj.thetwi - obj.Bi);

            c4 = Cy * obj.dt * dvy_dthetf * dthetf_dmagvi * dmagvi_datani * datani_dthetwi * dthetwi_dFsteeri;
            c5 = Cy * obj.dt * dvy_dthetf * dthetf_dBi * dBi_dthetwi * dthetwi_dFsteeri;
            c6 = Cy * obj.dt * dvy_dthetf * dthetf_dthetwi * dthetwi_dFsteeri;

            dCy_dFsteer = c1 + c2 + c3 + c4 + c5 + c6;

%             lrate = 0.0000001;
%             obj.turner.kp = obj.turner.kp - lrate * dCy_dFsteer * obj.turner.errors(end);
%             obj.turner.ki = obj.turner.ki - lrate * dCy_dFsteer * obj.turner.integ;
%             obj.turner.kd = obj.turner.kd - lrate * dCy_dFsteer * (obj.turner.errors(end) - obj.turner.errors(end-2));


	        % thetf, thetw (calculated from magv, B, thetw)
            obj.thetf = obj.thetf + obj.dt * deg_((obj.magv / obj.L) * cosd(obj.B) * tand(obj.thetw));       % thetf = integral((v/L)*cos(B)*tan(thetw))
            obj.thetf = obj.bound_ang(obj.thetf);
            obj.thetw = obj.thetw + obj.dt^2 * 0.5 * obj.Fsteer / obj.m_swhl;
            obj.bound_thetw();

            obj.thetwi = obj.thetw;
            obj.Bi = obj.B;
            obj.magvi = obj.magv;
            obj.Fcari = obj.Fcar;

        end


        function autotune_turner(obj)
            ntsteps = 50;
            amp = 3.4;
            cas = obj.make_cases(ntsteps);

            lrate = 0.00000001;
            tnn = NN(1,[5,2,1],["relu","tanh", "N/A"], lrate);
            failcnt = 0;

            obj.refspd.reset(); obj.refspd.set_ks([0.022 0.02 0]); obj.turner.set_ks([0 0 0]); vset = 1;
            nepochs = 1000;
            for ep = 1:nepochs
                for i = 1:2:size(cas, 1)
                    obj.refspd.reset();
                    obj.turner.reset();
                    obj.init_locn();
                    if ep > nepochs-1
                        disp('hi');
                    end
                    for t = 1:ntsteps
                        obj.scaleup_Fpedals(obj.refspd.push(norm(obj.v), vset));

                        a_in = [obj.x(2) amp MLangle(obj.thetf) sind(obj.thetf) obj.magv];
                        obj.scaleup_Fsteer(    tnn.forward(a_in)   );
%                         obj.scaleup_Fsteer( obj.turner.push(obj.x(2), amp) );

                        % TERMINATE
                        if (obj.thetf > 40 || obj.thetf < -40) && ep < nepochs
                            failcnt = failcnt + 1;
%                             disp(failcnt);
                            break
                        end

                        % BACK PROP
                        supx = cas(i,t);
                        supy = cas(i+1,t) -0.25;
                        dCy_dFsteer = obj.update_xvthet4(supx, supy);
                        lrate = 0.0000001;
%                         obj.turner.kp = obj.turner.kp - lrate * dCy_dFsteer * obj.turner.errors(end);
%                         obj.turner.ki = obj.turner.ki - lrate * dCy_dFsteer * obj.turner.integ;
%                         obj.turner.kd = obj.turner.kd - lrate * dCy_dFsteer * (obj.turner.errors(end) - obj.turner.errors(end-2));

%                         disp(dCy_dFsteer);
                        tnn.backward_given_dcda(dCy_dFsteer);

                        if ep == nepochs
                            disp([dCy_dFsteer obj.x(2)]);
                            display_cars([obj]);
                            pause(0.1);
                        end
                    end
                end
            end
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

        function mag_e = get_ctrack_er(obj, lancoor)
            vtl = [0 lancoor - obj.x(2) 0]';
            thet_vtl = obj.math.vect2ang(vtl);
            dthet = 0;
            if thet_vtl < 90 || thet_vtl > 270
                dthet = obj.bound_ang( abs(obj.thetf - 90 - thet_vtl) );
            else
                dthet = obj.bound_ang( abs(obj.thetf + 90 - thet_vtl) );
            end
            mag_e = norm(vtl) / cosd(dthet);

            mag_e = obj.x(2) - lancoor;

            if (obj.x(2) > lancoor)
                mag_e = -abs(mag_e);
            else
                mag_e = abs(mag_e);
            end
        end

        function followlane(obj, lancoor)
            k = 1;

            e_ = obj.get_ctrack_er(lancoor);
            val = obj.bound_ang(0 + atand(k * e_ / norm(obj.v)));
            if abs(val) < 45
%                 obj.scaleup_Fsteer( obj.turner.push(obj.thetw, e_));
                obj.thetw = e_;
            elseif val >= 45
%                 obj.scaleup_Fsteer( obj.turner.push(obj.thetw, 45));
                obj.thetw = 45;
            else
%                 obj.scaleup_Fsteer( obj.turner.push(obj.thetw, -45));
                obj.thetw = -45;
            end
        end

        function update(obj)
            obj.update_xvthet4(1, 1);
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
	        obj.ethresh = 0;
%             obj.x = [-40 0 0]'; obj.v = []
            for i = 1:60
                obj.x(1) = -randi(204) - 42;
                obj.v(1) = randi(64);
                n0.v(1) = randi(64);
                n0.x(1) = 0;
                obj.refspd.reset(); obj.safetail.reset(); n0.refspd.reset(); n0.safetail.reset();
                obj.mode = 0;

                for t = 1:ntimesteps
                    obj.cruise_ctl(n0, vset0, vset1);
    
%                     if obj.mode == 1
%                         a = [norm(obj.v) norm(n0.v)       norm(obj.x - n0.x) - obj.L/2 - n0.L/2 - (obj.calc_brakedist() - n0.approx_brakedist_motorcycle())];
%                         temp = sprintf('%f,', a);
%                         temp(end) = [];
%                         disp(temp);
%                         break;
%                     end
    
                    obj.update_xvthet();
                    n0.update_xvthet();
%                     obj.boxplot();
%                     n0.boxplot();
    
%                     % display cars
%                     obj.show("r");
%                     hold on;
%                     n0.show("b");
%                     hold off;
%                     axis([-100 1500 -100 100]);
%                     pause(0.1);
    
                    %------- change vset(lead) -------
                    if (t > 20)
                        vset0 = 0;
                    end
    
                    %------- detect collision -------
                    if obj.detect_collision(n0) == 1
                        break
                    end
    
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
            obj.ethresh = (1.1 * norm(obj.v));

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

        %%
                %% CRUISE CONTROL DRIVER FUNCTIONS
        function autotuneCruiseCtrl(obj, n0, vset0, vset1, ntimesteps)
	        obj.ethresh = 0;
%             obj.x = [-40 0 0]'; obj.v = []
            for i = 1:10
                obj.x(1) = -randi(204) - 42;
                obj.v(1) = randi(64);
                n0.v(1) = randi(64);
                n0.x(1) = 0;
                obj.refspd.reset(); obj.safetail.reset(); n0.refspd.reset(); n0.safetail.reset();
                obj.safetail.set_ks([0 0 0]);
                obj.mode = 0;
                obj.recording = 0; obj.tcnt = 0;
                suc = 0;

                for t = 1:ntimesteps
                    ampmode  = obj.autotune_cruise_ctl(n0, vset0, vset1);
    
%                     if obj.mode == 1
%                         a = [norm(obj.v) norm(n0.v)       norm(obj.x - n0.x) - obj.L/2 - n0.L/2 - (obj.calc_brakedist() - n0.approx_brakedist_motorcycle())];
%                         temp = sprintf('%f,', a);
%                         temp(end) = [];
%                         disp(temp);
%                         break;
%                     end
    
                    obj.update_xvthet();
                    n0.update_xvthet();

                    if obj.recording == 1
                        
                        if obj.tcnt == 1
                            obj.safetail.reset();
                        end
                        disp([obj.safetail.kp obj.safetail.ki obj.safetail.kd]);
                        obj.tcnt = obj.tcnt + 1;
                        mtd = n0.a(1) - (obj.Ft - obj.Fb - obj.Froll) / obj.mass;
                        C = obj.mode_supd(obj.tcnt) - mtd;
                        lrate = 0.000000003;
                        obj.safetail.kp = obj.safetail.kp - lrate * C*(1/obj.mass)*(27000/2)*obj.safetail.errors(end);
                        obj.safetail.ki = obj.safetail.ki - lrate * C*(1/obj.mass)*(27000/2)*obj.safetail.integ;
                        obj.safetail.kd = obj.safetail.kd - lrate * C*(1/obj.mass)*(27000/2)*(obj.safetail.errors(end) - obj.safetail.errors(end-2));
                        obj.mode_rec(obj.tcnt) = ampmode;
                        obj.Frec(obj.tcnt) = 150*(((obj.Ft-obj.Fb) + 20000)/27000 - 0.5);

%                         display_cars([obj, n0]);
%                         pause(0.1);
                        if obj.tcnt > numel(obj.mode_supd) || obj.tcnt > 13
                            obj.recording = 0; obj.tcnt = 0;
                            plot(obj.mode_supd);
                            hold on;
                            plot(obj.mode_exp);
                            hold on;
                            plot(obj.mode_rec);
                            hold on;
                            plot(obj.Frec);
                            hold off;
                            axis([0 100 -200 200]);
                            suc = 1;

                        end
                    end
                    if suc == 1
                        display_cars([obj, n0]);
                        pause(0.1);
                    end



%                     obj.boxplot();
%                     n0.boxplot();
    
%                     % display cars
%                     obj.show("r");
%                     hold on;
%                     n0.show("b");
%                     hold off;
%                     axis([-100 1500 -100 100]);
%                     pause(0.1);
    
                    %------- change vset(lead) -------
                    if (t > 20)
                        vset0 = 0;
                    end
    
                    %------- detect collision -------
                    if obj.detect_collision(n0) == 1
                        break
                    end
    
                end
            end
        end


        function ampmode = autotune_cruise_ctl(obj, n0, vset0, vset1)      % normal cruise ctl

            %------- NODE 1 -------
            followdist = norm(obj.x - n0.x) - obj.L/2 - n0.L/2;

            dsafe = obj.calc_brakedist() - n0.approx_brakedist_motorcycle();% - n0.calc_brakedist();
            obj.ethresh = (1.1 * norm(obj.v));

            % historesis
            want_mode = (followdist - dsafe) <0;
            if norm(obj.v) < 2 && norm(n0.v) < 2 && followdist < 8
                want_mode = 2;
            end



            % ADDED ------------------------
            if want_mode == 1 && obj.mode == 0
                obj.tcnt = 0;
                obj.recording = 1;
                t_ = 1:1:30;
                tbrake = obj.mass * obj.v(1) / (0.5 * obj.Fb_max);
%                 b = log(-log(-1 + 1/0.99)) - tbrake;
                obj.ethresh = 0.7 * norm(obj.v);
                a_ = -.5*(followdist - dsafe - obj.ethresh);
%                 obj.mode_supd = a_ - a_ .* 2.7128.^(t_+b);      % fixme: cant use this kinda sigmoid
%                 obj.mode_exp = 1 ./ (1 + 2.7128 .^ -(2.7128.^(t_ + b))) * a_ - 0.99*a_ ;
                b = -2.9;
                m = (-0.5*log(0.01/0.99) - b) / tbrake;
                obj.mode_exp = -0.99.*a_ + a_ .* (2.7128.^(m.*t_ + b) - 2.7128.^(-(m.*t_ + b))) ./ (2.7128.^(m.*t_ + b) + 2.7128.^(-(m.*t_ + b)));

                obj.mode_supd = -2*(m.*t_ + b) * m^2;

                plot(obj.mode_supd);
                hold on;
                plot(obj.mode_exp);
                hold off;
                axis([-50 50 -50 50]);

            end
            % END OF ADDED ------------------------


            obj.historesis(want_mode);

	        % run both PID controllers
	        [Ft_ref, Fb_ref] = obj.scaleup_Fpedals2(obj.refspd.push(norm(obj.v), vset1));	% calc Ft_ref, Fb_ref
	        [Ft_safe, Fb_safe] = obj.scaleup_Fpedals2(obj.safetail.push(0,followdist - dsafe - obj.ethresh));		% calc Ft_safe, Fb_safe
            ampmode = followdist - dsafe - obj.ethresh;
            [Ft_crawl, Fb_crawl] = obj.scaleup_Fpedals2(obj.crawl.push(0,followdist - dsafe - 5));     % calc Ft_crawl, Fb_crawl

            % generate value for Fcar, based on cruise control mode
	        obj.choose_Fmode(Ft_ref, Fb_ref, Ft_safe, Fb_safe, Ft_crawl, Fb_crawl);		% choose Ft, Fb based on mode & ethresh

%             disp([ followdist norm(obj.v) obj.Ft obj.Fb obj.mode ]);

            %------- NODE 0 -------
            n0.scaleup_Fpedals(n0.refspd.push(norm(n0.v), vset0));

        end

















                %% CRUISE CONTROL DRIVER FUNCTIONS
        function runCruiseCtrl_(obj, n0, vset0, vset1, ntimesteps)
	        obj.ethresh = 0;
            obj.x = [-40 0 0]';
            
            for t = 1:ntimesteps
                obj.cruise_ctl_(n0, vset0, vset1);

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
                if (t > 20)
                    vset0 = 10;
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



