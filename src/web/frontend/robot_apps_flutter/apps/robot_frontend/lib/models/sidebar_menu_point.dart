import 'package:flutter/material.dart';

class SidebarMenuPoint {

  SidebarMenuPoint({
    required this.title,
    required this.icon,
    required this.userGroupWidgets,
  });
  final String title;
  final IconData icon;
  final Map<String, Widget Function()> userGroupWidgets;

  Widget getWidgetForUserGroup(String userGroup) {
    if (userGroupWidgets.containsKey(userGroup)) {
      return userGroupWidgets[userGroup]!();
    } else {
      return const Center(
        child: Text('Keine Berechtigung'),
      );
    }
  }
}
