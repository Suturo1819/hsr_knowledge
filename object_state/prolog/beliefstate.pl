:- module(beliefstate,
    [
      gripper/1,
      new_perceived_at/4,
      hsr_existing_object_at/4,
      attach_object_to_gripper/1,
      release_object_from_gripper/0,
      belief_object_at_location/3,
      belief_class_of/2
    ]).

:- rdf_db:rdf_register_ns(hsr_objects, 'http://www.semanticweb.org/suturo/ontologies/2018/10/objects#', [keep(true)]).
:- rdf_db:rdf_register_ns(robocup, 'http://knowrob.org/kb/robocup.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(srdl2_comp, 'http://knowrob.org/kb/srdl2-comp.owl#', [keep(true)]).

:- rdf_meta
    gripper(+),
    new_perceived_at(r,+,+,r),
    hsr_existing_object_at(r,+,+,r),
    attach_object_to_gripper(r),
    release_object_from_gripper,
    belief_object_at_location(r,+,+),
    belief_class_of(r,r).

gripper(Gripper) :-
    belief_existing_objects([Gripper|_]), ! .

gripper(Gripper) :-
    rdf_instance_from_class(knowrob:'EnduringThing-Localized', belief_state, Gripper),
    rdf_assert(Gripper, rdf:type, owl:'NamedIndividual', belief_state),
    rdf_assert(Gripper, knowrob:'frameName', hand_palm_link, belief_state).

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

attach_object_to_gripper(Instance) :-
    rdf_retractall(Instance, hsr_objects:'supportedBy', _),
    gripper(Gripper),
    rdf_assert(Instance, hsr_objects:'supportedBy', Gripper),
    object_frame_name(Instance, InstanceFrame),
    object_frame_name(Gripper, GripperFrame),
    tf_lookup_transform(GripperFrame, InstanceFrame, PoseTerm),
    owl_instance_from_class(knowrob:'Pose', [pose=PoseTerm], Pose),
    transform_data(Pose,(Translation, Rotation)),
    belief_at_update(Instance, [GripperFrame, _, Translation, Rotation]).

release_object_from_gripper :-
    gripper(Gripper),
    objects_on_surface(Instances, Gripper),
    member(Instance, Instances),
    object_frame_name(Instance, InstanceFrame),
    tf_lookup_transform(map, InstanceFrame, PoseTerm),
    owl_instance_from_class(knowrob:'Pose', [pose=PoseTerm], Pose),
    transform_data(Pose,(Translation, Rotation)),
    belief_at_update(Instance, [map, _, Translation, Rotation]).


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