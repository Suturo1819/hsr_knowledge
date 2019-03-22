:- module(static_preset,
    [
      belief_static_object_at/8
    ]).

:- rdf_meta
    belief_static_object_at(?,r,r,r,r,r,?,?).

belief_static_object_at(Name, ObjectType, Transform, Width, Height, Depth, CadModel, Instance) :-
    belief_new_object(ObjectType, Instance),
    object_assert_dimensions(Instance, Depth, Width, Height),
    atom(CadModel),
    rdf_assert(Instance, knowrob:pathToCadModel, literal(type(xsd:string, CadModel))),
    belief_at_update(Instance, Transform).