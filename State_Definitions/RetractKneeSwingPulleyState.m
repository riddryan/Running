classdef RetractKneeSwingPulleyState
    %A class to label the states in the state vector
    
    properties
        knee = struct('Angle', 0,'AngleDot',0);
        foot = struct('Angle', 0,'AngleDot',0);
        pulley = struct('Angle',0,'AngleDot',0);
        order=[1 4 2 5 3 6]; % The order that these properties read left->right & up-> down appear in the state vector
    end
    
    methods
        
        function [this] = RetractKneeSwingPulleyState(input)
           if nargin == 0
              return; 
           end
           
           if isnumeric(input)
              this = RetractKneeSwingPulleyState;
              this = this.setFromVector(input);
              return;
           end
            
           this = input;
        end
        
        function [statevec] = getVector(this)
            count = 0;
            
            
           order = this.order;
           firstnames = fieldnames(this);
           firstnames = firstnames(1:end-1);
           for i = 1:length(firstnames)
               secondnames = fieldnames(this.(firstnames{i}));
               for j = 1:length(secondnames)
                   count = count+1;
                   statevec(order(count)) = this.(firstnames{i}).(secondnames{j});
               end
           end
           
        end
        
        function this = setFromVector(this,statevec)
           count = 0;
           
           order = this.order;
           firstnames = fieldnames(this);
           firstnames = firstnames(1:end-1);
           
           for i = 1:length(firstnames)
               secondnames = fieldnames(this.(firstnames{i}));
               for j = 1:length(secondnames)
                   count = count+1;
                   this.(firstnames{i}).(secondnames{j}) = statevec(order(count));
               end
           end 
            
        end
        
    end
    
end

