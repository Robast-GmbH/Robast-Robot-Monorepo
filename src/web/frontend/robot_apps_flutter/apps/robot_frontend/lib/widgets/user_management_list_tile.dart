import 'package:flutter/material.dart';
import 'package:middleware_api_utilities/middleware_api_utilities.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/models/provider/map_provider.dart';
import 'package:robot_frontend/models/provider/user_provider.dart';

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

  String? station;
  String? room;

  late bool isPatient;
  late bool isStaff;
  late bool isAdmin;

  @override
  void initState() {
    super.initState();
    title = widget.user.title == '' ? null : widget.user.title;
    firstNameController = TextEditingController(text: widget.user.firstName);
    lastNameController = TextEditingController(text: widget.user.lastName);

    station = widget.user.station == '' ? null : widget.user.station;
    room = widget.user.room == '' ? null : widget.user.room;

    isPatient = widget.user.userGroups.contains('PATIENT');
    isStaff = widget.user.userGroups.contains('STAFF');
    isAdmin = widget.user.userGroups.contains('ADMIN');
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
                      ),),
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
                child: DropdownButton<String>(
              isExpanded: true,
              hint: const Text('Station auswählen'),
              value: station,
              onChanged: (value) {
                if (station == value) return;
                setState(() {
                  station = value ?? '';
                  room = null;
                });
              },
              items: Provider.of<MapProvider>(context)
                  .locations
                  .keys
                  .map(
                    (availableStation) => DropdownMenuItem<String>(
                      value: availableStation,
                      alignment: Alignment.center,
                      child: Text(availableStation),
                    ),
                  )
                  .toList(),
            ),),
            const SizedBox(
              width: 16,
            ),
            Expanded(
              child: DropdownButton<String>(
                isExpanded: true,
                value: room,
                hint: const Text('Raum auswählen'),
                disabledHint: const Text(''),
                onChanged: (value) => setState(() => room = value ?? ''),
                items: station?.isEmpty ?? true
                    ? []
                    : Provider.of<MapProvider>(context)
                        .locations[station]!
                        .map(
                          (availableRoom) => DropdownMenuItem<String>(
                            value: availableRoom,
                            alignment: Alignment.center,
                            child: Text(availableRoom),
                          ),
                        )
                        .toList(),
              ),
            ),
            const SizedBox(
              width: 32,
            ),
            Expanded(
              flex: 2,
              child: Row(
                children: [
                  Checkbox(value: isPatient, onChanged: (value) => setState(() => isPatient = value ?? false)),
                  const Text('Patient'),
                  const SizedBox(
                    width: 16,
                  ),
                  Checkbox(value: isStaff, onChanged: (value) => setState(() => isStaff = value ?? false)),
                  const Text('Angestellte/r'),
                  const SizedBox(
                    width: 16,
                  ),
                  Checkbox(value: isAdmin, onChanged: (value) => setState(() => isAdmin = value ?? false)),
                  const Text('Admin'),
                ],
              ),
            ),
            const SizedBox(
              width: 32,
            ),
            IconButton(
              color: Colors.white,
              onPressed: () {
                final userGroups = [
                  if (isPatient) 'PATIENT',
                  if (isStaff) 'STAFF',
                  if (isAdmin) 'ADMIN',
                ];
                Provider.of<UserProvider>(context, listen: false).updateUser(
                  updatedUser: User(
                    id: widget.user.id,
                    title: title ?? '',
                    firstName: firstNameController.text,
                    lastName: lastNameController.text,
                    station: station ?? '',
                    room: room ?? '',
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
