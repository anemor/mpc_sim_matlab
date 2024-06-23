classdef FSM
    %simple State Machine to keep track of Hopper movement

    enumeration
        IDLE, ARMED, ASCENT, HOVER, LANDING, TOUCHDOWN, ABORT
    end
end