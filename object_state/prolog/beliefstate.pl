:- module(beliefstate,
    [
      gripper/1,
      new_perceived_at/4,
      hsr_existing_object_at/4,
      attach_object_to_gripper/1,
      release_object_from_gripper/0,
      select_surface/2,
      belief_object_at_location/3,
      belief_class_of/2,
      hsr_belief_at_update/2
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
    select_surface(r,?),
    belief_object_at_location(r,+,+),
    belief_class_of(r,r),
    hsr_belief_at_update(r,r).

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
    hsr_belief_at_update(Instance, Transform).

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
    transform_data(Pose,([X,Y,Z], Rotation)),
    hsr_belief_at_update(Instance, [map, _, [X,Y,Z], Rotation]),
    rdf_retractall(Instance, hsr_objects:'supportedBy', _),
    select_surface([X,Y,Z], Surface),
    rdf_assert(Instance, hsr_objects:'supportedBy', Surface).

select_surface([X,Y,Z], Surface) :-
    table_surface(Table),
    object_frame_name(Table, UrdfName),
    string_concat('environment/', UrdfName, TableFrame),
    hsr_lookup_transform(map, TableFrame, [TX,TY,_], _),
    hsr_lookup_transform(map, environment/shelf_base_center, [SX,SY,_], _),
    DTable is sqrt((TX-X)*(TX-X) + (TY-Y)*(TY-Y)),
    DShelf is sqrt((SX-X)*(SX-X) + (SY-Y)*(SY-Y)),
    ((DShelf < DTable, shelf_floor_at_height(Z, Surface));
    rdf_equal(Surface, Table)), ! .


% Object placed between two groups
hsr_belief_at_update(Instance, Transform) :-
    findall(NearbyGroup, (
        not(rdf_equal(Instance, NearbyObject)),
        hsr_existing_object_at(Transform, 0.2, NearbyObject),
        rdf_has(NearbyObject, hsr_objects:'inGroup', NearbyGroup)
        ), [GroupA, GroupB]),
    owl_instance_from_class(hsr_objects:'Group', NewGroup),
    (rdf_has(GroupMember, hsr_objects:'inGroup', GroupA);
     rdf_has(GroupMember, hsr_objects:'inGroup', GroupB)),
    rdf_retractall(GroupMember, hsr_objects:'inGroup', _),
    rdf_assert(GroupMember, hsr_objects:'inGroup', NewGroup),
    rdf_assert(Instance, hsr_objects:'inGroup', NewGroup),
    belief_at_update(Instance, Transform), !.

% Object placed nearby a group
hsr_belief_at_update(Instance, Transform) :-
    not(rdf_equal(Instance, NearbyObject)),
    hsr_existing_object_at(Transform, 0.2, NearbyObject),
    rdf_has(NearbyObject, hsr_objects:'inGroup', NearbyGroup),
    rdf_assert(Instance, hsr_objects:'inGroup', NearbyGroup),
    belief_at_update(Instance, Transform), !.

% No groups nearby
hsr_belief_at_update(Instance, Transform) :-
    owl_instance_from_class(hsr_objects:'Group', Group),
    rdf_assert(Instance, hsr_objects:'inGroup', Group),
    belief_at_update(Instance, Transform), !.


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
