// ignore: constant_identifier_names
enum ModuleType { manual_drawer, electric_drawer }

class DrawerModule {
  final int moduleID;
  final int drawerID;
  final ModuleType type;
  final int size;
  final String robotName;
  final int position;
  final String status;
  final String label;

  DrawerModule({
    required this.moduleID,
    required this.drawerID,
    required this.type,
    required this.size,
    required this.robotName,
    required this.position,
    required this.status,
    required this.label,
  });

  static DrawerModule fromJson(Map<String, dynamic> data) {
    return DrawerModule(
        moduleID: data["module_id"],
        drawerID: data["drawer_id"],
        type: ModuleType.values
            .firstWhere((element) => element.name == data["type"]),
        size: data["size"],
        robotName: data["robot_name"] ?? "rb_theron",
        position: data["pos"],
        status: data["is_open"] ? "Opened" : "Closed",
        label: data["label"] ?? "Dinge");
  }
}
