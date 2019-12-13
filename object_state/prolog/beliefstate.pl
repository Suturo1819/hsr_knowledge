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
      hsr_belief_at_update/2,
      merge_object_into_group/1,
      group_shelf_objects/0,
      group_mean_pose/3
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
    hsr_belief_at_update(r,r),
    merge_object_into_group(r),
    group_shelf_objects,
    group_mean_pose(r,?,?).

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
    rdf_assert(Instance, hsr_objects:'supportedBy', Surface),
    group_shelf_objects.

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

% No groups nearby
hsr_belief_at_update(Instance, Transform) :-
    kb_create(hsr_objects:'Group', Group, _{graph:groups}),
%    owl_instance_from_class(hsr_objects:'Group', Group),
    rdf_assert(Instance, hsr_objects:'inGroup', Group),
    belief_at_update(Instance, Transform).

merge_object_into_group(Instance) :-
    current_object_pose(Instance, Transform),
    findall(NearbyObj, (
        hsr_existing_object_at(Transform, 0.15, NearbyObj)),
        [Obj|Rest]),
    rdf_has(Obj, hsr_objects:'inGroup', WG),
    member(Other, Rest),
    rdf_retractall(Other, hsr_objects:'inGroup', _),
    rdf_assert(Other, hsr_objects:'inGroup', WG).


group_shelf_objects :-
    all_objects_in_whole_shelf(Objs),
    member(Obj, Objs),
    current_object_pose(Obj, Transform),
    hsr_existing_object_at(Transform, 0.15, NearbyObj),
    rdf_has(Obj, hsr_objects:'inGroup', Group1),
    rdf_has(NearbyObj, hsr_objects:'inGroup', Group2),
    not(rdf_equal(Obj, NearbyObj)),
    not(rdf_equal(Group1, Group2)),
    rdf_has(Member, hsr_objects:'inGroup', Group1),
    rdf_retractall(Member, hsr_objects:'inGroup', _),
    rdf_assert(Member, hsr_objects:'inGroup', Group2),
    not(group_shelf_objects).

group_mean_pose(Group, Transform, Rotation) :-
    findall(X, (
        rdf_has(Member, hsr_objects:'inGroup', Group),
        current_object_pose(Member, [_,_,[X,_,_],_])),
    Xs),
    findall(Y, (
        rdf_has(Member, hsr_objects:'inGroup', Group),
        current_object_pose(Member, [_,_,[_,Y,_],_])),
    Ys),
    findall(Z, (
        rdf_has(Member, hsr_objects:'inGroup', Group),
        current_object_pose(Member, [_,_,[_,_,Z],_])),
    Zs),
    sumlist(Xs, Xtotal),
    sumlist(Ys, Ytotal),
    sumlist(Zs,Ztotal),
    length(Xs, L),
    Xmean is Xtotal / L,
    Ymean is Ytotal / L,
    Zmean is Ztotal / L,
    Transform = [Xmean, Ymean, Zmean],
    once(rdf_has(Member, hsr_objects:'inGroup', Group)),
    object_current_surface(Member, Surface),
    surface_pose_in_map(Surface, [_, Rotation]),
    object_frame_name(Group, Frame),
    object_pose_update(Group, ['map', Frame, Transform, Rotation]).

%% Add these predicates because they are not exported in the corresponding modules
belief_object_at_location(ObjectId, NewPose, Dmax) :-
    object_pose(ObjectId, OldPose),
    transform_close_to(NewPose, OldPose, Dmax).

belief_class_of(Obj, ObjType) :-
    % nothing to do if current classification matches beliefs
    kb_type_of(Obj, ObjType), !.

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
