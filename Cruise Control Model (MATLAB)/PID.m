classdef PID < handle
    properties
        errors
        integ=0
        kp,ki,kd;
        dt;

    end
    
    methods
        function obj = PID(kp,ki,kd,dt, length)
            obj.kp = kp; obj.ki = ki; obj.kd = kd;
            obj.errors = zeros(1,length);
            obj.dt = dt;
        end
        function update_ers(obj, er)
            obj.integ = obj.integ + obj.dt * (er - obj.errors(1));
            obj.errors = obj.errors(2:end);
            obj.errors(end+1) = er;
        end

        function Fo = push(obj, val, val_exp)
            er = val_exp - val;
            obj.update_ers(er);
            Fo = obj.kp * er + obj.ki * obj.integ + obj.kd * (er - obj.errors(end-2));
            if Fo > 1
                Fo = 1;
            elseif Fo < -1
                Fo = -1;
            end
        end

        function reset(obj)
            obj.integ = 0; obj.errors = zeros(1, numel(obj.errors));
        end
        function set_ks(obj, ks)
            obj.kp = ks(1); obj.ki = ks(2); obj.kd = ks(3);
        end
        
    end
end
