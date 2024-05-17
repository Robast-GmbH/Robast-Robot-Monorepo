// ignore: constant_identifier_names
enum ModuleType { manual_drawer, electric_drawer }

class DrawerModule {
  DrawerModule({
    required this.moduleID,
    required this.drawerID,
    required this.type,
    required this.size,
    required this.robotName,
    required this.position,
    required this.isOpen,
    required this.label,
  });

  DrawerModule.fromJson({required Map<String, dynamic> data})
      : moduleID = data['module_id'] as int,
        drawerID = data['drawer_id'] as int,
        type = ModuleType.values.firstWhere((element) => element.name == data['type']),
        size = data['size'] as int,
        robotName = data['robot_name'] as String? ?? 'rb_theron',
        position = data['pos'] as int,
        isOpen = data['is_open'] as bool,
        label = data['label'] as String? ?? 'Dinge';

  final int moduleID;
  final int drawerID;
  final ModuleType type;
  final int size;
  final String robotName;
  final int position;
  final bool isOpen;
  final String label;
}
