class DropoffPlaceSubmoduleAssignment {
  String? dropoffPlaceID;
  String? submoduleID;

  bool isValid() {
    return dropoffPlaceID != null && submoduleID != null;
  }
}
