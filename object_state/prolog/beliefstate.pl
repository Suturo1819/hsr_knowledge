:- module(beliefstate,
    [
      new_perceived_at/4,
      hsr_existing_object_at/4,
      belief_object_at_location/3,
      belief_class_of/2
    ]).

:- rdf_db:rdf_register_ns(hsr_objects, 'http://www.semanticweb.org/suturo/ontologies/2018/10/objects#', [keep(true)]).
:- rdf_db:rdf_register_ns(robocup, 'http://knowrob.org/kb/robocup.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(srdl2_comp, 'http://knowrob.org/kb/srdl2-comp.owl#', [keep(true)]).

:- rdf_meta
    new_perceived_at(r,+,+,r),
    hsr_existing_object_at(r,+,+,r),
    belief_object_at_location(r,+,+),
    belief_class_of(r,r).

new_perceived_at(ObjType, Transform, Threshold, Instance) :-
    hsr_existing_object_at(ObjType, Transform, Threshold, Instance),
    belief_class_of(Instance, ObjType), !.

new_perceived_at(ObjType, Transform, _, Instance) :-
    belief_new_object(ObjType, Instance),
    belief_at_update(Instance, Transform).

hsr_existing_object_at(_, Transform, Threshold, Instance) :-
    rdf(Instance, rdf:type, owl:'NamedIndividual', belief_state),
    rdfs_individual_of(Instance, hsr_objects:'Robocupthings'),
    belief_object_at_location(Instance, Transform, Threshold), !.

%% Add these predicates because they are not exported in the corresponding modules
belief_object_at_location(ObjectId, NewPose, Dmax) :-
    belief_at_id(ObjectId, OldPose),
    transform_close_to(NewPose, OldPose, Dmax).

belief_class_of(Obj, ObjType) :-
    % nothing to do if current classification matches beliefs
    rdfs_type_of(Obj, ObjType), !.

belief_class_of(Obj, NewObjType) :-
    current_time(Now),
     ignore(once((
        % withdraw old beliefs about object type
        rdfs_type_of(Obj, CurrObjType),
         rdfs_subclass_of(CurrObjType, Parent),
        rdfs_subclass_of(NewObjType, Parent),
         assert_temporal_part_end(Obj, rdf:type, CurrObjType, Now, belief_state)
    ))),
    assert_temporal_part(Obj, rdf:type, nontemporal(NewObjType), Now, belief_state).