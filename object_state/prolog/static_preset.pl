:- module(static_preset,
    [
      belief_static_object_at/8

    ]).

:- rdf_meta belief_static_object_at(?,r,r,r,r,r,?,?).

belief_static_object_at(Name, ObjectType, Transform, Width, Height, Depth, CadModel, Instance) :-
    false.