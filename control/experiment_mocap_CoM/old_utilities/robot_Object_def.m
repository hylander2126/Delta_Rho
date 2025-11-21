classdef robot < matlab.mixin.SetGet
    properties
        N
        num
        K
        u_r
        exit
        ID
        u_data
        u_type
        u_converter
    end
    methods
        %__________________________________________________________________
        % robot constructor('Property','value,...)
        function [obj] = robot(varargin)
            for i=1:2:nargin-1
                obj.(varargin{i}) = varargin{i+1};
            end
        end
    end
end