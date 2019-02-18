
:- module(object_state,
    [
      create_object/2
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
:- rdf_meta create_object(r,?).

create_object(KnowrobType, Instance):-
	owl_subclass_of(KnowrobType, knowrob:'EnduringThing-Localized'),
	belief_new_object(KnowrobType, Instance).