import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:shared_data_models/shared_data_models.dart';
import 'package:web_frontend/models/provider/user_provider.dart';
import 'package:web_frontend/pages/user_creation_page.dart';
import 'package:web_frontend/widgets/user_list_tile.dart';

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
    return Scaffold(
      appBar: AppBar(
          title: Row(
        children: [
          Icon(Icons.group),
          SizedBox(
            width: 16,
          ),
          Text('Nutzermanagement'),
        ],
      )),
      floatingActionButton: FloatingActionButton(
        onPressed: () async {
          final didCreateUser = await Navigator.push(
            context,
            MaterialPageRoute<bool>(
              builder: (context) => UserCreationPage(),
            ),
          );
          if (didCreateUser ?? false) {
            setState(() {
              loadUsers = Provider.of<UserProvider>(context, listen: false).getUsers();
            });
          }
        },
        child: Icon(Icons.person_add),
      ),
      body: Center(
        child: FutureBuilder<List<User>>(
            future: loadUsers,
            builder: (context, snapshot) {
              if (snapshot.connectionState != ConnectionState.done) {
                return const Center(child: CircularProgressIndicator());
              }
              final users = snapshot.data!;
              return ListView(
                children: users
                    .map((user) => UserListTile(
                          user: user,
                          onDelete: () => setState(() {
                            loadUsers = Provider.of<UserProvider>(context, listen: false).getUsers();
                          }),
                          onUpdate: () => setState(() {
                            loadUsers = Provider.of<UserProvider>(context, listen: false).getUsers();
                          }),
                        ))
                    .toList(),
              );
            }),
      ),
    );
  }
}
