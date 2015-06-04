classdef SoftParRunnerState
    %A class to label the states in the state vector
    
    properties
        pelvis = struct('x', 0, 'y', 0, 'xDot', 0, 'yDot',0);
        stanceLeg = struct('Angle', 0, 'Length',0,'AngleDot', 0,'LengthDot',0);
        softparallel = struct('stretch',0,'stretchDot',0);
        
        order=[4 5 9 10 1 2 6 7 3 8]; % The order that these properties read left->right & up-> down appear in the state vector
    end
    
    methods
        
        function [this] = SoftParRunnerState(input)
           if nargin == 0
              return; 
           end
           
           if isnumeric(input)
              this = SoftParRunnerState;
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

