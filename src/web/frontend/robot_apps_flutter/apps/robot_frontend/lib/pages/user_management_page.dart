import 'package:flutter/material.dart';
import 'package:middleware_api_utilities/middleware_api_utilities.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/models/provider/user_provider.dart';
import 'package:robot_frontend/widgets/custom_scaffold.dart';
import 'package:robot_frontend/widgets/rounded_button.dart';
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
              padding: const EdgeInsets.all(16),
              child: Column(
                children: [
                  Expanded(
                    child: ListView.builder(
                      itemCount: snapshot.data!.length,
                      itemBuilder: (context, index) => UserManagementListTile(
                        user: snapshot.data![index],
                        onUserUpdate: () {
                          setState(() {
                            loadUsers = Provider.of<UserProvider>(context, listen: false).getUsers();
                          });
                        },
                      ),
                    ),
                  ),
                  const SizedBox(
                    height: 16,
                  ),
                  Padding(
                    padding: const EdgeInsets.symmetric(horizontal: 8),
                    child: RoundedButton(
                        onPressed: () async {
                          await Provider.of<UserProvider>(context, listen: false).createUser(
                            newUser: User(
                              id: '',
                              nfcID: '',
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
                        color: Colors.black.withOpacity(0.2),
                        child: const Padding(
                          padding: EdgeInsets.symmetric(vertical: 8),
                          child: Icon(
                            Icons.add,
                            size: 48,
                          ),
                        ),),
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
