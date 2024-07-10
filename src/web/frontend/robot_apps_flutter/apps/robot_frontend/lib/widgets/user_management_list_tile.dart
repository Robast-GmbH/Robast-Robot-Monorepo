import 'package:flutter/material.dart';
import 'package:middleware_api_utilities/middleware_api_utilities.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/models/controller/location_selection_controller.dart';
import 'package:robot_frontend/models/controller/user_groups_selection_controller.dart';
import 'package:robot_frontend/models/provider/user_provider.dart';
import 'package:robot_frontend/widgets/location_selector.dart';
import 'package:robot_frontend/widgets/user_groups_selector.dart';

class UserManagementListTile extends StatefulWidget {
  const UserManagementListTile({required this.user, required this.onDeletePressed, super.key});
  final User user;
  final VoidCallback onDeletePressed;
  @override
  State<UserManagementListTile> createState() => _UserManagementListTileState();
}

class _UserManagementListTileState extends State<UserManagementListTile> {
  String? title;
  late final TextEditingController firstNameController;
  late final TextEditingController lastNameController;

  final locationSelectionController = LocationSelectionController();
  final userGroupsSelectionController = UserGroupsSelectionController();

  @override
  void initState() {
    super.initState();
    title = widget.user.title == '' ? null : widget.user.title;
    firstNameController = TextEditingController(text: widget.user.firstName);
    lastNameController = TextEditingController(text: widget.user.lastName);

    locationSelectionController
      ..setStation(widget.user.station == '' ? null : widget.user.station)
      ..setRoom(widget.user.room == '' ? null : widget.user.room);

    userGroupsSelectionController
      ..isPatient = widget.user.userGroups.contains('PATIENT')
      ..isStaff = widget.user.userGroups.contains('STAFF')
      ..isAdmin = widget.user.userGroups.contains('ADMIN');
  }

  @override
  Widget build(BuildContext context) {
    return Card(
      color: Colors.white.withOpacity(0.5),
      child: Padding(
        padding: const EdgeInsets.all(8),
        child: Row(
          children: [
            Expanded(
              flex: 2,
              child: Row(
                children: [
                  Expanded(
                    flex: 2,
                    child: DropdownButton<String>(
                      isExpanded: true,
                      value: title,
                      onChanged: (value) => setState(() => title = value ?? ''),
                      items: Provider.of<UserProvider>(context)
                          .availableTitles
                          .map(
                            (title) => DropdownMenuItem<String>(
                              value: title,
                              alignment: Alignment.center,
                              child: Text(title),
                            ),
                          )
                          .toList(),
                    ),
                  ),
                  const SizedBox(
                    width: 16,
                  ),
                  Expanded(
                    flex: 3,
                    child: TextField(
                      controller: firstNameController,
                    ),
                  ),
                  const SizedBox(
                    width: 16,
                  ),
                  Expanded(
                    flex: 3,
                    child: TextField(
                      controller: lastNameController,
                    ),
                  ),
                ],
              ),
            ),
            const SizedBox(
              width: 32,
            ),
            Expanded(
              child: LocationSelector(
                controller: locationSelectionController,
              ),
            ),
            const SizedBox(
              width: 32,
            ),
            Expanded(
              flex: 2,
              child: UserGroupsSelector(
                controller: userGroupsSelectionController,
              ),
            ),
            const SizedBox(
              width: 32,
            ),
            IconButton(
              color: Colors.white,
              onPressed: () {
                final userGroups = userGroupsSelectionController.selectionAsStringList();
                Provider.of<UserProvider>(context, listen: false).updateUser(
                  updatedUser: User(
                    id: widget.user.id,
                    title: title ?? '',
                    firstName: firstNameController.text,
                    lastName: lastNameController.text,
                    station: locationSelectionController.station ?? '',
                    room: locationSelectionController.room ?? '',
                    userGroups: userGroups,
                  ),
                );
              },
              icon: const Icon(Icons.save_alt),
            ),
            IconButton(
              color: Colors.white,
              onPressed: () {
                widget.onDeletePressed();
              },
              icon: const Icon(Icons.delete),
            ),
          ],
        ),
      ),
    );
  }
}
