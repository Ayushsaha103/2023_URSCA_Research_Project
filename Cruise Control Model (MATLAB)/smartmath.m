

% function mv = mag(v)
%     mv = norm(v);
% end
classdef smartmath
    methods
        function obj = smartmath()
        end
        % convert a given magnitude and angle into a vector
        function vect = ang2vect(obj, mag, thet)
            vect = [mag*cos(rad_(thet)) mag*sin(rad_(thet)) 0]';
        end
        % send back the angle of a given vector
        function thet = vect2ang(obj, v)
           	thet = deg_(acos(v(1) / ((v(1)^2 + v(2)^2)^0.5)));
            if v(2) < 0
                thet = -thet +360;
            end
        end
        function thet = min_ang_btwn(obj, thetv, thetf)
            thet = min(360 - max(thetf, thetv) + min(thetf, thetv), abs(thetf - thetv));
        end
        function rd = proj(obj, v,u)
            s = (v(1) * u(1) + v(2) * u(2) + v(3) * u(3)) / (obj.norm3D(v)^2);
            rd = [0 0 0]';
            for i=1:3
                rd(i) = v(i) * s;
            end
        end
        function rd = comp(obj, v, u)
            rd = (v(1) * u(1) + v(2) * u(2) + v(3) * u(3)) / norm(v);
        end
        function val = norm3D(obj, v)
            val = (v(1)^2 + v(2)^2 + v(3)^2)^0.5;
        end
        function vec = ang2vect3D(obj, mag, thet_xy, phi_z)
            vec = [0 0 0]';
            vec(1) = mag * sin(rad_(90-phi_z)) * cos(rad_(thet_xy));
            vec(2) = mag * sin(rad_(90-phi_z)) * sin(rad_(thet_xy));
            vec(3) = mag * cos(rad_(90-phi_z));
        end
        function ans_ = vect_along(obj, u, v)            % return 1 if v is along u
            ang_btwn = acosd(dot(u,v) / (norm(u)*norm(v)));
            ans_ = 1;
            if ang_btwn > 90 || ang_btwn < -90
                ans_ = 0;
            end
        end
        function new_integ = RK4(obj, last3_yprime, old_integ)
            k1 = 2 * last3_yprime(1);
            k2 = 2 * last3_yprime(2);
            k3 = 2 * last3_yprime(2);
            k4 = 2 * last3_yprime(3);
            new_integ = old_integ + (k1 + 2*k2 + 2*k3 + k4)/6;
        end
    end
end


