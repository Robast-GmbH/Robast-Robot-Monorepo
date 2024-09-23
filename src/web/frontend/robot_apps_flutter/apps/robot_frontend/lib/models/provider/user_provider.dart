import 'package:flutter/material.dart';
import 'package:middleware_api_utilities/middleware_api_utilities.dart';
import 'package:shared_data_models/shared_data_models.dart';

class UserProvider extends ChangeNotifier {
  final availableTitles = [
    '',
    'Dr.',
    'Prof.',
    'Prof. Dr.',
  ];

  final _middlewareApiUtilities = MiddlewareApiUtilities();

  Future<List<User>> getUsers() async {
    return _middlewareApiUtilities.users.getUsers();
  }

  Future<User?> getUser({required String id}) async {
    return _middlewareApiUtilities.users.getUser(id: id);
  }

  Future<User?> updateUser({required User updatedUser}) async {
    return _middlewareApiUtilities.users.updateUser(updatedUser: updatedUser);
  }

  Future<User?> createUser({required User newUser}) async {
    return _middlewareApiUtilities.users.createUser(newUser: newUser);
  }

  Future<bool> deleteUser({required String id}) async {
    return _middlewareApiUtilities.users.deleteUser(id: id);
  }

  Future<bool> readAndAssignUserNFC({
    required String robotName,
    required String userID,
  }) async {
    return _middlewareApiUtilities.users.readAndAssignUserNFC(robotName: robotName, userID: userID);
  }

  Future<bool> setUserSession({required String robotName, required String userID}) async {
    return _middlewareApiUtilities.users.setUserSession(robotName: robotName, userID: userID);
  }

  Future<User?> getUserSession({required String robotName}) async {
    return _middlewareApiUtilities.users.getUserSession(robotName: robotName);
  }

  Future<bool> tryStartUserSession({
    required String robotName,
    List<String> requiredUserIDs = const [],
    List<String> requiredUserGroups = const [],
  }) async {
    return _middlewareApiUtilities.users.tryStartUserSession(
      robotName: robotName,
      requiredUserIDs: requiredUserIDs,
      requiredUserGroups: requiredUserGroups,
    );
  }

  Future<bool> endUserSession({required String robotName}) async {
    return _middlewareApiUtilities.users.endUserSession(robotName: robotName);
  }

  Future<bool> writeNFC({required String robotName, required String nfcData}) async {
    return _middlewareApiUtilities.nfc.writeNFC(robotName: robotName, nfcData: nfcData);
  }

  Future<String> readNFC({required String robotName}) async {
    return _middlewareApiUtilities.nfc.readNFC(robotName: robotName);
  }
}
