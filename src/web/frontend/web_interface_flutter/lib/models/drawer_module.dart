class DrawerModule {
  final int moduleID;
  final int drawerID;

  DrawerModule({
    required this.moduleID,
    required this.drawerID,
  });

  static DrawerModule fromJson(Map<String, dynamic> data) {
    return DrawerModule(
      moduleID: data["id"],
      drawerID: data["drawer_id"],
    );
  }
}
