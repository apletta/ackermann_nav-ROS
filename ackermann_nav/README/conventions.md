# Conventions
### Naming and data type conventions for nodes, topics, and msgs in the ackermann_nav package.


- All topics must have at least a `Header header` field. This is so they can have a time stamp assigned to them so they can be used with ApproximateTime message_filter to allow nodes to subscribe to multiple topics. 
- Messages are capitalized according to `NameMsg`, with their full name being `ackermann_nav::NameMsg`.
- Message types are abbreviated in node files by specifying `typedef ackermann_nav::NameMsg myNameMsg` at the top of the file. This is intended to improve code readability. 
- all node code is in c++
- a "float64" is a "double" in c++
