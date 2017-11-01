classdef node
   properties
      position
      orientation
      states
      measurement
      estimates
      measurement_hat
      matrixC
   end
   
   methods
      function obj = MyClass(val)
         if nargin > 0
            obj.Prop = val;
         end
      end
   end
end