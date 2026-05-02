// kOS main script for Falcon9 planner bridge.
// It writes vehicle + launch config to 0:/send.txt, then a standalone
// C++ bridge program (planner_comm_bridge.exe) can read that file and
// launch falcon9_gui_planner.exe.
//
// Optional part tags in craft (recommended, not required):
//   Stage-1 engines: S1_ENG
//   Stage-2 engines: S2_ENG
//   Stage-1 parts:   S1
//   Stage-2 parts:   S2
//   Payload parts:   PAYLOAD
//
// Units written to send.txt:
//   mass: kg, thrust: kN, isp: s, angles: deg, distance: km.

SET SEND_FILE TO "0:/send.txt".
IF EXISTS(SEND_FILE) { DELETEPATH(SEND_FILE). }.

// Mission constants (can be adjusted)
SET SHIP_DOWNRANGE_KM TO 1000.
SET ASCENT_LOSSES_MPS TO 1500.
SET MAX_Q_LIMIT_KPA TO 40.
SET STAGE1_RESERVE_RATIO TO 0.08.

FUNCTION PART_WET_MASS_KG {
  PARAMETER P.
  RETURN P:MASS * 1000.
}.

FUNCTION PART_DRY_MASS_KG {
  PARAMETER P.
  RETURN P:DRYMASS * 1000.
}.

FUNCTION SUM_MASS_BY_TAG {
  PARAMETER TAG_NAME.
  PARAMETER WANT_DRY.

  LOCAL SUM_KG IS 0.
  LIST PARTS IN ALL_PARTS.
  FOR P IN ALL_PARTS {
    IF P:TAG = TAG_NAME {
      IF WANT_DRY = 1 {
        SET SUM_KG TO SUM_KG + PART_DRY_MASS_KG(P).
      } ELSE {
        SET SUM_KG TO SUM_KG + PART_WET_MASS_KG(P).
      }.
    }.
  }.
  RETURN SUM_KG.
}.

FUNCTION WRITE_KV {
  PARAMETER KEY.
  PARAMETER VAL.
  LOG KEY + "=" + ROUND(VAL, 6) TO SEND_FILE.
}.

FUNCTION WRITE_KVS {
  PARAMETER KEY.
  PARAMETER VALTXT.
  LOG KEY + "=" + VALTXT TO SEND_FILE.
}.

FUNCTION IS_S1_ENGINE_NAME {
  PARAMETER PARTNAME.
  IF PARTNAME = "TE.19.F9.S1.Engine" { RETURN 1. }.
  IF PARTNAME = "PMB.F9.Merlin1Dplusplus" { RETURN 1. }.
  RETURN 0.
}.

FUNCTION IS_S2_ENGINE_NAME {
  PARAMETER PARTNAME.
  IF PARTNAME = "TE.19.F9.S2.Engine" { RETURN 1. }.
  IF PARTNAME = "PMB.F9.Merlin1DVplus" { RETURN 1. }.
  RETURN 0.
}.

FUNCTION IS_STAGE_SEPARATOR_NAME {
  PARAMETER PARTNAME.
  IF PARTNAME = "Decoupler.3" { RETURN 1. }.
  RETURN 0.
}.

FUNCTION IS_PAYLOAD_SEPARATOR_NAME {
  PARAMETER PARTNAME.
  IF PARTNAME = "Decoupler.2" { RETURN 1. }.
  RETURN 0.
}.

FUNCTION IS_S1_PART {
  PARAMETER P.
  LOCAL N IS P:NAME.
  IF IS_S1_ENGINE_NAME(N) = 1 { RETURN 1. }.
  IF N = "TE.19.F9.S1.Tank" { RETURN 1. }.
  IF N = "TE.19.F9.S1.Interstage" { RETURN 1. }.
  IF N = "KRE-FalconLegMk2-M" { RETURN 1. }.
  IF N = "Grid Fin M Titanium" { RETURN 1. }.
  IF N = "TE2.19.F9.CGT" { RETURN 1. }.
  IF N = "KK.SPX.F93.interstage" { RETURN 1. }.
  IF N = "KK.SPX.F93.S1tank" { RETURN 1. }.
  IF N = "PMB.Rockomax16.BW" { RETURN 1. }.
  IF N = "KK.SPX.F9.Octaweb" { RETURN 1. }.
  IF N = "Vandy.F9" { RETURN 1. }.
  IF N = "KK.F9demo.S1RCS" { RETURN 1. }.
  IF N = "Grid Fin L Titanium" { RETURN 1. }.
  IF N = "KK.SPX.F9LandingLeg" { RETURN 1. }.
  IF N = "miniFuelTank" { RETURN 1. }.
  RETURN 0.
}.

FUNCTION IS_S2_PART {
  PARAMETER P.
  LOCAL N IS P:NAME.
  IF IS_S2_ENGINE_NAME(N) = 1 { RETURN 1. }.
  IF N = "TE.19.F9.S2.Tank" { RETURN 1. }.
  IF N = "TE.F9.S2.RCS" { RETURN 1. }.
  IF N = "TE.F9.Fairing.Adapter" { RETURN 1. }.
  IF N = "TE.19.F9.Fairing" { RETURN 1. }.
  IF N = "KK.SPX.F93.S2tank" { RETURN 1. }.
  IF N = "KK.SpX.Mvac.skirt" { RETURN 1. }.
  IF N = "linearRcs" { RETURN 1. }.
  IF N = "KK.SPX.FalconPayloadFairing" { RETURN 1. }.
  IF N = "KK.SpX.FRH" { RETURN 1. }.
  IF N = "Decoupler.3" { RETURN 1. }.
  IF N = "Decoupler.2" { RETURN 1. }.
  IF N = "asasmodule1-2" { RETURN 1. }.
  IF N = "KK.SPX.F9.1875mmPA" { RETURN 1. }.
  IF N = "KK.SPX.Falcon9.FC" { RETURN 1. }.
  RETURN 0.
}.

FUNCTION IS_IGNORE_PART {
  PARAMETER P.
  IF P:NAME = "TE.Ghidorah.Erector" { RETURN 1. }.
  IF P:NAME = "launchClamp1" { RETURN 1. }.
  RETURN 0.
}.

FUNCTION IS_TRUE_TEXT {
  PARAMETER T.
  IF T = "True" { RETURN 1. }.
  IF T = "true" { RETURN 1. }.
  IF T = "1" { RETURN 1. }.
  RETURN 0.
}.

FUNCTION BRANCH_HAS_ENGINE {
  PARAMETER ROOT.
  PARAMETER WANT_S1.
  PARAMETER WANT_S2.

  IF WANT_S1 = 1 {
    IF IS_S1_ENGINE_NAME(ROOT:NAME) = 1 { RETURN 1. }.
    IF ROOT:PARTSNAMED("TE.19.F9.S1.Engine"):LENGTH > 0 { RETURN 1. }.
    IF ROOT:PARTSNAMED("PMB.F9.Merlin1Dplusplus"):LENGTH > 0 { RETURN 1. }.
  }.

  IF WANT_S2 = 1 {
    IF IS_S2_ENGINE_NAME(ROOT:NAME) = 1 { RETURN 1. }.
    IF ROOT:PARTSNAMED("TE.19.F9.S2.Engine"):LENGTH > 0 { RETURN 1. }.
    IF ROOT:PARTSNAMED("PMB.F9.Merlin1DVplus"):LENGTH > 0 { RETURN 1. }.
  }.

  RETURN 0.
}.

FUNCTION IS_DESCENDANT_OF {
  PARAMETER P.
  PARAMETER ROOT.

  LOCAL CUR IS P.
  UNTIL CUR:HASPARENT = FALSE {
    IF CUR = ROOT { RETURN 1. }.
    SET CUR TO CUR:PARENT.
  }.

  IF CUR = ROOT { RETURN 1. }.
  RETURN 0.
}.

FUNCTION IS_IN_CHILD_BRANCH_OF {
  PARAMETER P.
  PARAMETER SEP.

  FOR C IN SEP:CHILDREN {
    IF IS_DESCENDANT_OF(P, C) = 1 { RETURN 1. }.
  }.

  RETURN 0.
}.

// Part-name based engine model lookup (works even if engine module is not enabled).
FUNCTION GET_ENGINE_MODEL_THRUST_KN {
  PARAMETER PARTNAME.
  IF PARTNAME = "TE.19.F9.S1.Engine" { RETURN 7607. }.
  IF PARTNAME = "PMB.F9.Merlin1Dplusplus" { RETURN 845.222222. }.
  IF PARTNAME = "TE.19.F9.S2.Engine" { RETURN 981. }.
  IF PARTNAME = "PMB.F9.Merlin1DVplus" { RETURN 981. }.
  RETURN 0.
}.

FUNCTION GET_ENGINE_MODEL_ISP_S {
  PARAMETER PARTNAME.
  IF PARTNAME = "TE.19.F9.S1.Engine" { RETURN 282.0. }.
  IF PARTNAME = "PMB.F9.Merlin1Dplusplus" { RETURN 282.0. }.
  IF PARTNAME = "TE.19.F9.S2.Engine" { RETURN 348.0. }.
  IF PARTNAME = "PMB.F9.Merlin1DVplus" { RETURN 348.0. }.
  RETURN 0.
}.

// ---- Parts and mass collection (no tags needed) ----
LIST PARTS IN ALL_PARTS.
LOCAL STAGE_SEPARATOR IS SHIP:ROOTPART.
LOCAL PAYLOAD_SEPARATOR IS SHIP:ROOTPART.
LOCAL STAGE_SEPARATOR_FOUND IS 0.
LOCAL PAYLOAD_SEPARATOR_FOUND IS 0.

FOR P IN ALL_PARTS {
  IF STAGE_SEPARATOR_FOUND = 0 {
    IF IS_STAGE_SEPARATOR_NAME(P:NAME) = 1 {
      FOR C IN P:CHILDREN {
        IF BRANCH_HAS_ENGINE(C, 1, 0) = 1 {
          SET STAGE_SEPARATOR TO P.
          SET STAGE_SEPARATOR_FOUND TO 1.
        }.
      }.
    }.
  }.

  IF PAYLOAD_SEPARATOR_FOUND = 0 {
    IF IS_PAYLOAD_SEPARATOR_NAME(P:NAME) = 1 {
      LOCAL CHILD_HAS_MAIN_ENGINE IS 0.
      FOR C IN P:CHILDREN {
        IF BRANCH_HAS_ENGINE(C, 1, 1) = 1 {
          SET CHILD_HAS_MAIN_ENGINE TO 1.
        }.
      }.
      IF CHILD_HAS_MAIN_ENGINE = 0 {
        SET PAYLOAD_SEPARATOR TO P.
        SET PAYLOAD_SEPARATOR_FOUND TO 1.
      }.
    }.
  }.
}.

LOCAL S1_WET_KG IS 0.
LOCAL S1_DRY_KG IS 0.
LOCAL S2_WET_KG IS 0.
LOCAL S2_DRY_KG IS 0.
LOCAL PAYLOAD_KG IS 0.

FOR P IN ALL_PARTS {
  IF IS_IGNORE_PART(P) = 1 {
    // skip non-flight support parts
  } ELSE {
    LOCAL WET_KG IS PART_WET_MASS_KG(P).
    LOCAL DRY_KG IS PART_DRY_MASS_KG(P).
    LOCAL MATCHED_S1 IS 0.
    LOCAL MATCHED_PAYLOAD IS 0.

    IF STAGE_SEPARATOR_FOUND = 1 {
      IF IS_IN_CHILD_BRANCH_OF(P, STAGE_SEPARATOR) = 1 {
        SET MATCHED_S1 TO 1.
      }.
    } ELSE {
      IF IS_S1_PART(P) = 1 {
        SET MATCHED_S1 TO 1.
      }.
    }.

    IF PAYLOAD_SEPARATOR_FOUND = 1 {
      IF IS_IN_CHILD_BRANCH_OF(P, PAYLOAD_SEPARATOR) = 1 {
        SET MATCHED_PAYLOAD TO 1.
      }.
    }.

    IF MATCHED_S1 = 1 {
      SET S1_WET_KG TO S1_WET_KG + WET_KG.
      SET S1_DRY_KG TO S1_DRY_KG + DRY_KG.
    } ELSE {
      IF MATCHED_PAYLOAD = 1 {
        SET PAYLOAD_KG TO PAYLOAD_KG + WET_KG.
      } ELSE {
        IF STAGE_SEPARATOR_FOUND = 1 {
          IF PAYLOAD_SEPARATOR_FOUND = 1 {
            SET S2_WET_KG TO S2_WET_KG + WET_KG.
            SET S2_DRY_KG TO S2_DRY_KG + DRY_KG.
          } ELSE IF IS_S2_PART(P) = 1 {
            SET S2_WET_KG TO S2_WET_KG + WET_KG.
            SET S2_DRY_KG TO S2_DRY_KG + DRY_KG.
          } ELSE {
            SET PAYLOAD_KG TO PAYLOAD_KG + WET_KG.
          }.
        } ELSE IF IS_S2_PART(P) = 1 {
          SET S2_WET_KG TO S2_WET_KG + WET_KG.
          SET S2_DRY_KG TO S2_DRY_KG + DRY_KG.
        } ELSE {
          SET PAYLOAD_KG TO PAYLOAD_KG + WET_KG.
        }.
      }.
    }.
  }.
}.

LOCAL S1_PROP_KG IS MAX(0, S1_WET_KG - S1_DRY_KG).
LOCAL S2_PROP_KG IS MAX(0, S2_WET_KG - S2_DRY_KG).

// ---- Engine parameters ----
LOCAL S1_THRUST_KN IS 0.
LOCAL S2_THRUST_KN IS 0.
LOCAL S1_ISP_S IS 282.
LOCAL S2_ISP_S IS 348.
LOCAL S1_ENGINE_COUNT IS 0.
LOCAL S2_ENGINE_COUNT IS 0.
LOCAL S1_MASS_SOURCE IS "parts_name_group".
LOCAL S2_MASS_SOURCE IS "parts_name_group".
LOCAL PAYLOAD_SOURCE IS "parts_name_group".
LOCAL S1_THRUST_SOURCE IS "default".
LOCAL S2_THRUST_SOURCE IS "default".
LOCAL S1_ISP_SOURCE IS "default".
LOCAL S2_ISP_SOURCE IS "default".

IF STAGE_SEPARATOR_FOUND = 1 {
  SET S1_MASS_SOURCE TO "decoupler_child_branch".
  IF PAYLOAD_SEPARATOR_FOUND = 1 {
    SET S2_MASS_SOURCE TO "upper_stack_minus_payload_branch".
    SET PAYLOAD_SOURCE TO "payload_decoupler_child_branch".
  } ELSE {
    SET S2_MASS_SOURCE TO "decoupler_plus_partname_fallback".
  }.
} ELSE IF PAYLOAD_SEPARATOR_FOUND = 1 {
  SET PAYLOAD_SOURCE TO "payload_decoupler_child_branch".
}.

FOR P IN ALL_PARTS {
  IF IS_S1_ENGINE_NAME(P:NAME) = 1 {
    SET S1_ENGINE_COUNT TO S1_ENGINE_COUNT + 1.
    SET S1_THRUST_KN TO S1_THRUST_KN + GET_ENGINE_MODEL_THRUST_KN(P:NAME).
    SET S1_ISP_S TO GET_ENGINE_MODEL_ISP_S(P:NAME).
    SET S1_THRUST_SOURCE TO "partname_engine_model".
    SET S1_ISP_SOURCE TO "partname_engine_model".
  } ELSE IF IS_S2_ENGINE_NAME(P:NAME) = 1 {
    SET S2_ENGINE_COUNT TO S2_ENGINE_COUNT + 1.
    SET S2_THRUST_KN TO S2_THRUST_KN + GET_ENGINE_MODEL_THRUST_KN(P:NAME).
    SET S2_ISP_S TO GET_ENGINE_MODEL_ISP_S(P:NAME).
    SET S2_THRUST_SOURCE TO "partname_engine_model".
    SET S2_ISP_SOURCE TO "partname_engine_model".
  }.
}.

// Fallbacks / safety defaults.
IF S2_DRY_KG <= 0 {
  SET S2_DRY_KG TO 4000.
  SET S2_MASS_SOURCE TO "default_dry".
}.
IF S2_PROP_KG <= 0 {
  SET S2_PROP_KG TO 92670.
  SET S2_MASS_SOURCE TO "default_prop".
}.
IF PAYLOAD_KG <= 0 {
  SET PAYLOAD_KG TO 15000.
  SET PAYLOAD_SOURCE TO "default_15000".
}.
IF S1_DRY_KG <= 0 {
  SET S1_DRY_KG TO 25600.
  SET S1_MASS_SOURCE TO "default_dry".
}.

// If stage mass tags are missing, estimate S1 prop from current full-stack wet mass.
IF S1_PROP_KG <= 0 {
  LOCAL STACK_WET_KG IS SHIP:MASS * 1000.
  LOCAL S1_WET_EST_KG IS STACK_WET_KG - (S2_DRY_KG + S2_PROP_KG + PAYLOAD_KG).
  IF S1_WET_EST_KG > S1_DRY_KG {
    SET S1_PROP_KG TO S1_WET_EST_KG - S1_DRY_KG.
    SET S1_MASS_SOURCE TO "estimated_from_stack".
    PRINT "stage1 mass source: estimated from stack mass.".
  } ELSE {
    SET S1_PROP_KG TO 395700.
    SET S1_MASS_SOURCE TO "default_395700".
    PRINT "stage1 mass source: fallback default.".
  }.
}.

IF S1_THRUST_KN <= 0 {
  SET S1_THRUST_KN TO 7607.
  SET S1_THRUST_SOURCE TO "default_7607".
}.
IF S2_THRUST_KN <= 0 {
  SET S2_THRUST_KN TO 981.
  SET S2_THRUST_SOURCE TO "default_981".
}.

// ---- Launch coordinates ----
LOCAL LAT_DEG IS SHIP:LATITUDE.
LOCAL LON_DEG IS SHIP:LONGITUDE.

// ---- Write planner config ----
WRITE_KV("payload_kg", PAYLOAD_KG).
WRITE_KV("s1_dry_kg", S1_DRY_KG).
WRITE_KV("s1_prop_kg", S1_PROP_KG).
WRITE_KV("s1_isp_s", S1_ISP_S).
WRITE_KV("s1_thrust_kN", S1_THRUST_KN).
WRITE_KV("s1_reserve", STAGE1_RESERVE_RATIO).

WRITE_KV("s2_dry_kg", S2_DRY_KG).
WRITE_KV("s2_prop_kg", S2_PROP_KG).
WRITE_KV("s2_isp_s", S2_ISP_S).
WRITE_KV("s2_thrust_kN", S2_THRUST_KN).
WRITE_KVS("s1_mass_source", S1_MASS_SOURCE).
WRITE_KVS("s2_mass_source", S2_MASS_SOURCE).
WRITE_KVS("payload_source", PAYLOAD_SOURCE).
WRITE_KV("stage_separator_found", STAGE_SEPARATOR_FOUND).
WRITE_KV("payload_separator_found", PAYLOAD_SEPARATOR_FOUND).
WRITE_KVS("s1_thrust_source", S1_THRUST_SOURCE).
WRITE_KVS("s2_thrust_source", S2_THRUST_SOURCE).
WRITE_KVS("s1_isp_source", S1_ISP_SOURCE).
WRITE_KVS("s2_isp_source", S2_ISP_SOURCE).
WRITE_KV("s1_engine_count", S1_ENGINE_COUNT).
WRITE_KV("s2_engine_count", S2_ENGINE_COUNT).
WRITE_KV("s1_wet_kg_raw", S1_WET_KG).
WRITE_KV("s1_dry_kg_raw", S1_DRY_KG).
WRITE_KV("s2_wet_kg_raw", S2_WET_KG).
WRITE_KV("s2_dry_kg_raw", S2_DRY_KG).
WRITE_KV("payload_kg_raw", PAYLOAD_KG).
WRITE_KVS("mass_unit_note", "ksp_ton_to_kg_x1000").

WRITE_KV("lat_deg", LAT_DEG).
WRITE_KV("launch_lon_deg", LON_DEG).
SET EARTH_ROTATION_ANGLE_DEG TO (TIME:SECONDS * 360 / SHIP:BODY:ROTATIONPERIOD).
WRITE_KV("earth_rotation_angle_deg", EARTH_ROTATION_ANGLE_DEG).
WRITE_KV("launch_window_half_width_min", 45).
WRITE_KV("ship_downrange_km", SHIP_DOWNRANGE_KM).
WRITE_KV("losses_mps", ASCENT_LOSSES_MPS).
WRITE_KV("q_limit_kpa", MAX_Q_LIMIT_KPA).

// Default orbit values for planner UI (still editable in GUI):
WRITE_KV("perigee_km", 200).
WRITE_KV("apogee_km", 200).
WRITE_KV("incl_deg", 28.5).

PRINT "planner send file written: " + SEND_FILE.
PRINT "lat/lon=" + ROUND(LAT_DEG,6) + "," + ROUND(LON_DEG,6).
PRINT "s1/s2 thrust(kN)=" + ROUND(S1_THRUST_KN,3) + "/" + ROUND(S2_THRUST_KN,3).
