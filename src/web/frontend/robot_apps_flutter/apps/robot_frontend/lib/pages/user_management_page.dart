import 'package:flutter/material.dart';
import 'package:middleware_api_utilities/middleware_api_utilities.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/models/provider/user_provider.dart';
import 'package:robot_frontend/widgets/custom_scaffold.dart';
import 'package:robot_frontend/widgets/user_management_list_tile.dart';

class UserManagementPage extends StatefulWidget {
  const UserManagementPage({super.key});

  @override
  State<UserManagementPage> createState() => _UserManagementPageState();
}

class _UserManagementPageState extends State<UserManagementPage> {
  late Future<List<User>> loadUsers;

  @override
  void initState() {
    super.initState();
    loadUsers = Provider.of<UserProvider>(context, listen: false).getUsers();
  }

  @override
  Widget build(BuildContext context) {
    return CustomScaffold(
      title: 'User Management',
      child: FutureBuilder<List<User>>(
        future: loadUsers,
        builder: (context, snapshot) {
          if (snapshot.connectionState != ConnectionState.done) {
            return const Center(child: CircularProgressIndicator());
          }
          return Center(
            child: Padding(
              padding: const EdgeInsets.all(64),
              child: Column(
                children: [
                  Expanded(
                    child: ListView(
                      children: snapshot.data!
                          .map<Widget>(
                            (user) => UserManagementListTile(
                              user: user,
                              onDeletePressed: () async {
                                await Provider.of<UserProvider>(context, listen: false).deleteUser(id: user.id);
                                setState(() {
                                  loadUsers = Provider.of<UserProvider>(context, listen: false).getUsers();
                                });
                              },
                            ),
                          )
                          .toList(),
                    ),
                  ),
                  const SizedBox(
                    height: 4,
                  ),
                  GestureDetector(
                    onTap: () async {
                      await Provider.of<UserProvider>(context, listen: false).createUser(
                        newUser: User(
                          id: '',
                          title: '',
                          firstName: '',
                          lastName: '',
                          station: '',
                          room: '',
                          userGroups: [],
                        ),
                      );
                      setState(() {
                        loadUsers = Provider.of<UserProvider>(context, listen: false).getUsers();
                      });
                    },
                    child: Card(
                      color: Colors.white.withOpacity(0.7),
                      child: const Center(
                        child: Padding(
                          padding: EdgeInsets.all(8),
                          child: Icon(
                            Icons.add,
                            size: 48,
                          ),
                        ),
                      ),
                    ),
                  ),
                ],
              ),
            ),
          );
        },
      ),
    );
  }
}
