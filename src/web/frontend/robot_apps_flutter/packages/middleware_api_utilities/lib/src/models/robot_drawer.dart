import 'package:middleware_api_utilities/src/models/drawer_address.dart';
import 'package:middleware_api_utilities/src/models/module_process.dart';

enum DrawerVariant { manual, electric }

class RobotDrawer {
  RobotDrawer({
    required this.robotName,
    required this.address,
    required this.position,
    required this.size,
    required this.variant,
    required this.itemsByCount,
    required this.reservedForIds,
    required this.reservedForGroups,
    required this.reservedForTask,
    required this.moduleProcess,
  });

  // Factory method to create a Drawer instance from a JSON object
  factory RobotDrawer.fromJson(Map<String, dynamic> json) {
    final drawerAddress = json['address'] as Map<String, dynamic>;
    return RobotDrawer(
      robotName: drawerAddress['robot_name'] as String,
      address: DrawerAddress(
        moduleID: drawerAddress['module_id'] as int,
        drawerID: drawerAddress['drawer_id'] as int,
      ),
      position: json['position'] as int,
      size: json['size'] as int,
      variant: DrawerVariant.values.firstWhere((element) => element.toString() == 'DrawerVariant.${json['variant']}'),
      itemsByCount: Map<String, int>.from(json['items_by_count'] as Map<String, dynamic>),
      reservedForTask: json['reserved_for_task'] as String,
      reservedForIds: List<String>.from(json['reserved_for_ids'] as List<dynamic>),
      reservedForGroups: List<String>.from(json['reserved_for_groups'] as List<dynamic>),
      moduleProcess: ModuleProcess.fromJson(json),
    );
  }
  final String robotName;
  final DrawerAddress address;
  final int position;
  final int size;
  final DrawerVariant variant;

  final Map<String, int> itemsByCount;
  final String reservedForTask;
  final List<String> reservedForIds;
  final List<String> reservedForGroups;
  final ModuleProcess moduleProcess;

  // Method to convert a Drawer instance to a JSON object
  Map<String, dynamic> toJson() {
    return {
      'address': {
        'robot_name': robotName,
        'module_id': address.moduleID,
        'drawer_id': address.drawerID,
      },
      'position': position,
      'size': size,
      'items_by_count': itemsByCount,
      'reserved_for_task': reservedForTask,
      'reserved_for_ids': reservedForIds,
      'reserved_for_groups': reservedForGroups,
      'module_process_status': moduleProcess.status,
      'module_process_type': moduleProcess.type,
      'module_process_items_by_change': moduleProcess.itemsByChange,
    };
  }

  bool isReserved() {
    return reservedForTask.isNotEmpty || reservedForIds.isNotEmpty || reservedForGroups.isNotEmpty;
  }
}
