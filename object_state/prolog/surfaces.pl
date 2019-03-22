:- module(surfaces,
    [
      object_surface/2,
      objects_on/2,
      assert_object_on/2,
      all_surfaces/1,
      shelf_floors/1,
      size_of/2
    ]).

:- rdf_db:rdf_register_ns(hsr_objects, 'http://www.semanticweb.org/suturo/ontologies/2018/10/objects#', [keep(true)]).
:- rdf_db:rdf_register_ns(robocup, 'http://knowrob.org/kb/robocup.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(srdl2_comp, 'http://knowrob.org/kb/srdl2-comp.owl#', [keep(true)]).

:- rdf_meta
	all_surfaces(?),
    shelf_floors(?),
    object_surface(?,?),
    objects_on(?,r),
    assert_object_on(r,r),
    size_of(r,?).

object_surface(Instance, Surface) :-
    rdf_has(Instance, hsr_objects:'supportedBy', Surface).

objects_on(Instances, Surface) :-
    findall(Instances,
        object_surface(Instances, Surface),
        Instances).

assert_object_on(Instance, Surface) :-
    rdf_assert(Instance, hsr_objects:'supportedBy', Surface).

all_surfaces(Surface) :-
    rdfs_individual_of(Surface, srdl2_comp:'UrdfLink').

shelf_floors(ShelfFloors) :-
    findall(ShelfFloors,
        all_surfaces(ShelfFloors),
        rdf_has(ShelfFloors, srdl2_comp:'urdfName', literal(UrdfName)),
        string_to_atom(AsString,UrdfName),
        sub_string(AsString, 0,11,_, 'shelf_floor'),
        ShelfFloors).

size_of(TFFrame, Size) :-
    rdf_has(Instance, srdl2_comp:'urdfName', literal(TFFrame)),
    rdf_has(Instance, srdl2_comp:'box_size', Size).

size_of(Instance, Size) :-
    rdf_has(Instance, srdl2_comp:'box_size', Size).