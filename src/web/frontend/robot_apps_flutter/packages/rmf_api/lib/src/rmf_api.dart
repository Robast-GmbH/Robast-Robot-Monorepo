import 'dart:convert';

import 'package:http/http.dart' as http;
import 'package:rmf_api/src/models/building_map.dart';
import 'package:rmf_api/src/models/task.dart';
import 'package:rmf_api/src/models/task_request.dart';

/// {@template rmf_api}
/// A Very Good Project created by Very Good CLI.
/// {@endtemplate}
class RmfApi {
  /// {@macro rmf_api}
  const RmfApi({required this.prefix});

  final String prefix;

  Future<BuildingMap> getBuildingMap() async {
    try {
      final response = await http.get(Uri.parse('$prefix/building_map'));
      if (response.statusCode == 200) {
        return BuildingMap.fromJson(jsonDecode(response.body) as Map<String, dynamic>);
      } else {
        throw Exception('Failed to fetch building map');
      }
    } catch (e) {
      throw Exception('Failed to fetch building map');
    }
  }

  Future<List<Task>> getTasks() async {
    try {
      final response = await http.get(Uri.parse('$prefix/tasks'));
      if (response.statusCode == 200) {
        final tasks = jsonDecode(response.body) as List<dynamic>;
        return tasks.map((e) => Task.fromJson(e as Map<String, dynamic>)).toList();
      } else {
        throw Exception('Failed to fetch tasks');
      }
    } catch (e) {
      throw Exception('Failed to fetch tasks');
    }
  }

  Future<void> dispatchDeliveryTask({required String pickup, required String dropoff, required String drawerID}) async {
    final task = DeliveryTaskRequest(pickup: pickup, dropoff: dropoff, drawerID: drawerID);
    await _dispatchTask(task: task);
  }

  Future<void> dispatchDropOffTask({required String dropoff, required String drawerID}) async {
    final task = DropOffTaskRequest(dropoff: dropoff, drawerID: drawerID);
    await _dispatchTask(task: task);
  }

  Future<void> dispatchPatrolTask({required List<String> places, required int rounds}) async {
    final task = PatrolTaskRequest(places: places, rounds: rounds);
    await _dispatchTask(task: task);
  }

  Future<void> _dispatchTask({required TaskRequest task}) async {
    try {
      final header = {'Content-Type': 'application/json'};
      final body = task.toJson();

      final response = await http.post(
        Uri.parse('$prefix/tasks/dispatch_task'),
        headers: header,
        body: jsonEncode(body),
      );

      if (response.statusCode != 200) {
        throw Exception('Failed to submit task');
      }
    } catch (e) {
      throw Exception('Failed to submit task');
    }
  }

  Future<String> interruptTask({required String taskID}) async {
    try {
      final header = {'Content-Type': 'application/json'};
      final body = {'type': 'interrupt_task_request', 'task_id': taskID, 'labels': <String>[]};
      final response = await http.post(
        Uri.parse('$prefix/tasks/interrupt_task?task_id=$taskID'),
        headers: header,
        body: jsonEncode(body),
      );
      if (response.statusCode != 200) {
        throw Exception('Failed to interrupt task');
      } else {
        return (jsonDecode(response.body) as Map<String, dynamic>)['token'] as String;
      }
    } catch (e) {
      throw Exception('Failed to interrupt task');
    }
  }

  Future<bool> resumeTask({required String taskID, required String token}) async {
    try {
      final header = {'Content-Type': 'application/json'};
      final body = {
        'type': 'resume_task_request',
        'for_task': taskID,
        'for_tokens': [token],
        'labels': <String>[],
      };
      final response = await http.post(
        Uri.parse('$prefix/tasks/resume_task?task_id=$taskID'),
        headers: header,
        body: jsonEncode(body),
      );
      if (response.statusCode != 200) {
        throw Exception('Failed to resume task');
      } else {
        return (jsonDecode(response.body) as Map<String, dynamic>)['success'] as bool;
      }
    } catch (e) {
      throw Exception('Failed to resume task');
    }
  }
}
