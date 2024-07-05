class User {
  User({
    required this.id,
    required this.title,
    required this.firstName,
    required this.lastName,
    required this.station,
    required this.room,
    required this.userGroups,
  });

  factory User.fromJson(Map<String, dynamic> json) {
    return User(
      id: json['id'] as String,
      title: json['title'] as String,
      firstName: json['first_name'] as String,
      lastName: json['last_name'] as String,
      station: json['station'] as String,
      room: json['room'] as String,
      userGroups: List<String>.from(json['user_groups'] as List<dynamic>),
    );
  }

  final String id;
  final String title;
  final String firstName;
  final String lastName;
  final String station;
  final String room;
  final List<String> userGroups;

  Map<String, dynamic> toJson() {
    return {
      'id': id,
      'title': title,
      'first_name': firstName,
      'last_name': lastName,
      'station': station,
      'room': room,
      'user_groups': userGroups,
    };
  }
}
