
:- module(object_state,
    [
      create_object/2,
      object_at/4,
      test_belief/0,
      spawn_on_table/0,
      clear/0,
      object_at_table/1,
      object_of_type/2,
      %semantically_closest_object/2,
      create_object_at/6,
      hsr_existing_objects/1
    ]).

/*:- use_module(library('lists')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/owl')).
:- use_module(library('knowrob/owl')).
:- use_module(library('knowrob/rdfs')).
:- use_module(library('knowrob/temporal')).
:- use_module(library('knowrob/transforms')).
:- use_module(library('knowrob/objects')).
*/

:- rdf_db:rdf_register_ns(hsr_objects, 'http://www.semanticweb.org/suturo/ontologies/2018/10/objects#', [keep(true)]).
:- rdf_db:rdf_register_ns(robocup, 'http://knowrob.org/kb/robocup.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(srdl2_comp, 'http://knowrob.org/kb/srdl2-comp.owl#', [keep(true)]).

:- rdf_meta
    create_object(r,?),
	object_at(r,r,r,?),
	object_at_table(?),
	object_of_type(r,?),
	test_belief,
	spawn_on_table,
	create_object_at(r,r,r,?,-,-),
	hsr_existing_objects(?).


hsr_existing_objects(Objects) :-
    belief_existing_objects(Objects, [hsr_objects:'Robocupthings']).


%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Interface predicates %%

object_at(ObjectType, Transform, Threshold, Instance) :-
	hsr_existing_objects(Objectlist),
	member(Instance, Objectlist),
	belief_existing_object_at(ObjectType, Transform, Threshold, Instance).

object_at_table(Instance) :-
	object_at(_, ['map', _, [1,0,0.8],[0,0,0,1]], 0.4, Instance).

object_of_type(ObjectType, Instance) :-
	belief_existing_objects(Instance, [ObjectType]).

%% Interface predicates %%
%%%%%%%%%%%%%%%%%%%%%%%%%%

%belief_new_object(hsr_objects:'Device', Inst),
%X = 0.7, atom_number(X_atom,X),
%object_assert_color(Inst, [X,X,X]),
%belief_at_update(Inst, [map, _, [1,1,1],[0,0,0,1]]).

create_object(ObjectType, Instance) :-
%	owl_subclass_of(ObjectType, knowrob:'EnduringThing-Localized'),
	belief_new_object(ObjectType, Instance).


create_object_at(ObjectType, Transform, Threshold, Instance, [Width, Depth, Height], [R,G,B,A]) :-
    new_perceived_at(ObjectType, Transform, Threshold, Instance),
    object_assert_dimensions(Instance, Width, Depth, Height),
    set_dimension_semantics(Instance, Width, Depth, Height),
    set_object_colour(Instance, [R,G,B,A]),
    hsr_existing_objects(Objects),
    belief_republish_objects(Objects).

set_dimension_semantics(Instance, _, _, Height) :-
    Height > 0.16,
    rdf_assert(Instance, hsr_objects:'size', 'tall').

set_dimension_semantics(Instance, Width, Depth, Height) :-
    Height < Width * 0.9,
    Height < Depth * 0.9,
    rdf_assert(Instance, hsr_objects:'size', 'flat').

set_dimension_semantics(Instance, Width, Depth, Height) :-
    ((Depth > Width * 2, Depth > Height * 2);
     (Width > Depth * 2, Width > Height * 2)),
    rdf_assert(Instance, hsr_objects:'size', 'long').

set_dimension_semantics(Instance, Width, Depth, Height) :-
    Volume is Width * Depth * Height * 1000,
    Volume < 0.6,
    rdf_assert(Instance, hsr_objects:'size', 'small').

set_dimension_semantics(Instance, Width, Depth, Height) :-
    Volume is Width * Depth * Height * 1000,
    Volume > 2.0,
    rdf_assert(Instance, hsr_objects:'size', 'big').

set_object_colour(Instance, [0.0, 0.0, 0.0, 0.0]) :-
    object_assert_color(Instance, [0.8, 0.8, 0.8, 0.8]),
    rdf_assert(Instance, hsr_objects:'colour', 'grey'), !.

set_object_colour(Instance, [R,G,B,_]) :-
    RConv is R/255,
    GConv is G/255,
    BConv is B/255,
    object_assert_color(Instance, [RConv,GConv,BConv,0.8]),
    set_colour_semantics(Instance, [RConv,GConv,BConv]).

set_colour_semantics(Instance, [0.0, 0.0, 0.0]) :-
    rdf_assert(Instance, hsr_objects:'colour', 'dark').

set_colour_semantics(Instance, [1.0, 0.0, 0.0]) :-
    rdf_assert(Instance, hsr_objects:'colour', 'red').

set_colour_semantics(Instance, [0.0, 1.0, 0.0]) :-
    rdf_assert(Instance, hsr_objects:'colour', 'green').

set_colour_semantics(Instance, [1.0, 1.0, 0.0]) :-
    rdf_assert(Instance, hsr_objects:'colour', 'yellow').

set_colour_semantics(Instance, [0.0, 0.0, 1.0]) :-
    rdf_assert(Instance, hsr_objects:'colour', 'dark-blue').

set_colour_semantics(Instance, [1.0, 0.0, 1.0]) :-
    rdf_assert(Instance, hsr_objects:'colour', 'violet').

set_colour_semantics(Instance, [0.0, 1.0, 1.0]) :-
    rdf_assert(Instance, hsr_objects:'colour', 'light-blue').

set_colour_semantics(Instance, [1.0, 1.0, 1.0]) :-
    rdf_assert(Instance, hsr_objects:'colour', 'bright').

%spawn_on_table :-
%	new_perceived_at(hsr_objects:'Cup', ['map', _, [1,0,0.8],[0,0,0,1]], 0.2, _).
%
%test_belief :-
%	belief_perceived_at(hsr_objects:'Cup', ['map', _, [1,0,0.8],[0,0,0,1]], 0.2, _),
%	object_at_table(Instance),
%	object_of_type(hsr_objects:'Cup', Instances),
%	member(Instance,Instances),
%	clear.

clear :-
	belief_forget.
