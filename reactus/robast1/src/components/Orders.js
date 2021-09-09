import PropTypes from 'prop-types'
import Order from './Order'


const Orders = ({orders, onDelete, onToggle}) => {     
        return (
                <>
                  {
                          orders.map((order) => (<Order 
                                key={order.id} 
                                task={order} 
                                onDelete = {onDelete} 
                                onToggle = {onToggle}/>))
                  }      
                </>
        )
}


Orders.protoTypes = {
        orders: PropTypes.object,
        onDelete: PropTypes.func,
        onToggle: PropTypes.func,
}

export default Orders
