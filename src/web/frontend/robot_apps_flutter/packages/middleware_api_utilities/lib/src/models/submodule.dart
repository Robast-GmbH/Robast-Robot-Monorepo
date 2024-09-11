import 'package:middleware_api_utilities/middleware_api_utilities.dart';

enum SubmoduleVariant { manual, electric, partial }

class Submodule {
  Submodule({
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

  // Factory method to create a Submodule instance from a JSON object
  factory Submodule.fromJson(Map<String, dynamic> json) {
    final submoduleAddress = json['address'] as Map<String, dynamic>;
    return Submodule(
      robotName: submoduleAddress['robot_name'] as String,
      address: SubmoduleAddress(
        moduleID: submoduleAddress['module_id'] as int,
        submoduleID: submoduleAddress['submodule_id'] as int,
      ),
      position: json['position'] as int,
      size: json['size'] as int,
      variant: SubmoduleVariant.values.firstWhere((element) => element.toString() == 'SubmoduleVariant.${json['variant']}'),
      itemsByCount: Map<String, int>.from(json['items_by_count'] as Map<String, dynamic>),
      reservedForTask: json['reserved_for_task'] as String,
      reservedForIds: List<String>.from(json['reserved_for_ids'] as List<dynamic>),
      reservedForGroups: List<String>.from(json['reserved_for_groups'] as List<dynamic>),
      moduleProcess: ModuleProcess.fromJson(json),
    );
  }
  final String robotName;
  final SubmoduleAddress address;
  final int position;
  final int size;
  final SubmoduleVariant variant;

  final Map<String, int> itemsByCount;
  final String reservedForTask;
  final List<String> reservedForIds;
  final List<String> reservedForGroups;
  final ModuleProcess moduleProcess;

  // Method to convert a Submodule instance to a JSON object
  Map<String, dynamic> toJson() {
    return {
      'address': {
        'robot_name': robotName,
        'module_id': address.moduleID,
        'submodule_id': address.submoduleID,
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

  bool checkUserAuth(User user) {
    return reservedForIds.contains(user.id) || reservedForGroups.any((group) => user.userGroups.contains(group));
  }

  String contentToString() {
    return itemsByCount.entries.map((entry) => '${entry.key}: ${entry.value}').join(', ');
  }
}
