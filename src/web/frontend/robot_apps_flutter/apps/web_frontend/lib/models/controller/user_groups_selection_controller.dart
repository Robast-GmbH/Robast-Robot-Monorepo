class UserGroupsSelectionController {
  bool isPatient = false;
  bool isStaff = false;
  bool isAdmin = false;

  List<String> selectionAsStringList() {
    return [
      if (isPatient) 'PATIENT',
      if (isStaff) 'STAFF',
      if (isAdmin) 'ADMIN',
    ];
  }
}
