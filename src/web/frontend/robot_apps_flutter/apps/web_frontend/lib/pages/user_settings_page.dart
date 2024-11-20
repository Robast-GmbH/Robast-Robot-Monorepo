import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:shared_data_models/shared_data_models.dart';
import 'package:web_frontend/models/provider/map_provider.dart';
import 'package:web_frontend/models/provider/user_provider.dart';
import 'package:web_frontend/pages/change_password_page.dart';
import 'package:web_frontend/validators.dart';
import 'package:web_frontend/widgets/custom_text_field.dart';

class UserSettingsPage extends StatefulWidget {
  const UserSettingsPage({required this.user, this.isAdminView = false, super.key});

  final User user;
  final bool isAdminView;

  @override
  State<UserSettingsPage> createState() => _UserSettingsPageState();
}

class _UserSettingsPageState extends State<UserSettingsPage> {
  final formKey = GlobalKey<FormState>();
  final firstNameController = TextEditingController();
  final lastNameController = TextEditingController();
  final mailController = TextEditingController();
  String? station;
  String? room;

  @override
  void initState() {
    super.initState();
    firstNameController.text = widget.user.firstName;
    lastNameController.text = widget.user.lastName;
    mailController.text = widget.user.mail ?? '';
    station = widget.user.station.isEmpty ? null : widget.user.station;
    room = widget.user.room.isEmpty ? null : widget.user.room;
  }

  @override
  Widget build(BuildContext context) {
    final mapProvider = Provider.of<MapProvider>(context, listen: false);
    return Scaffold(
      appBar: AppBar(
        title: const Row(
          children: [
            Icon(Icons.person),
            SizedBox(width: 8),
            Text('Nutzereinstellungen'),
          ],
        ),
      ),
      floatingActionButton: FloatingActionButton(
        onPressed: () async {
          if (formKey.currentState!.validate()) {
            final updatedUser = User(
              id: widget.user.id,
              nfcID: widget.user.nfcID,
              title: widget.user.title,
              firstName: firstNameController.text,
              lastName: lastNameController.text,
              mail: mailController.text,
              station: station ?? widget.user.station,
              room: room ?? widget.user.room,
              userGroups: widget.user.userGroups,
            );
            final wasSuccessful = await Provider.of<UserProvider>(context, listen: false).updateUser(updatedUser: updatedUser);
            if (wasSuccessful && context.mounted) {
              Navigator.pop(context, true);
            }
          }
        },
        child: const Icon(Icons.save),
      ),
      body: Padding(
        padding: const EdgeInsets.all(16),
        child: Form(
          key: formKey,
          child: ListView(
            children: [
              CustomTextField(
                label: 'Vorname',
                controller: firstNameController,
                validator: Validators.nameValidator,
              ),
              const SizedBox(height: 16),
              CustomTextField(
                label: 'Nachname',
                controller: lastNameController,
                validator: Validators.nameValidator,
              ),
              const SizedBox(height: 16),
              CustomTextField(
                label: 'E-Mail',
                controller: mailController,
                validator: Validators.mailValidator,
              ),
              if (mapProvider.roomsByStations.isNotEmpty) ...[
                const SizedBox(height: 16),
                Padding(
                  padding: const EdgeInsets.all(8),
                  child: Column(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      const Text('Station', style: TextStyle(color: Colors.white70, fontSize: 12)),
                      DropdownButton<String>(
                        isExpanded: true,
                        value: station,
                        hint: const Text('Station auswählen'),
                        dropdownColor: Colors.grey[800],
                        items: mapProvider.roomsByStations.keys
                            .map(
                              (station) => DropdownMenuItem<String>(
                                value: station,
                                child: Text(
                                  station,
                                  style: const TextStyle(color: Colors.white),
                                ),
                              ),
                            )
                            .toList(),
                        onChanged: (value) {
                          if (value != station) {
                            room = null;
                          }
                          setState(() {
                            station = value;
                          });
                        },
                      ),
                    ],
                  ),
                ),
                Padding(
                  padding: const EdgeInsets.all(8),
                  child: Column(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      const Text('Raum', style: TextStyle(color: Colors.white70, fontSize: 12)),
                      DropdownButton<String>(
                        isExpanded: true,
                        value: room,
                        dropdownColor: Colors.grey[800],
                        disabledHint: const Text('Bitte wählen Sie zuerst eine Station aus'),
                        hint: const Text('Raum auswählen'),
                        items: station == null
                            ? []
                            : mapProvider.roomsByStations[station]!
                                .map(
                                  (station) => DropdownMenuItem<String>(
                                    value: station,
                                    child: Text(
                                      station,
                                      style: const TextStyle(color: Colors.white),
                                    ),
                                  ),
                                )
                                .toList(),
                        onChanged: (value) {
                          setState(() {
                            room = value;
                          });
                        },
                      ),
                    ],
                  ),
                ),
              ],
              if (!widget.isAdminView) ...[
                const SizedBox(height: 16),
                ElevatedButton(
                  onPressed: () {
                    Navigator.push(
                      context,
                      MaterialPageRoute<ChangePasswordPage>(
                        builder: (context) => ChangePasswordPage(),
                      ),
                    );
                  },
                  child: const Padding(
                    padding: EdgeInsets.symmetric(vertical: 16),
                    child: Text('Passwort ändern'),
                  ),
                ),
              ],
            ],
          ),
        ),
      ),
    );
  }
}
