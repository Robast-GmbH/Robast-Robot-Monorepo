import PropTypes from 'prop-types'
import Order from './Order'


const RenameButton = ({button, name}) => {     
        return (
                <>
                  {
                          orders.map((order) => (<Order 
                                key={order.id} 
                                order={order} 
                                onDelete = {onDelete} 
                                onToggle = {onToggle}/>))
                  }      
                </>
        )
}


RenameButton.protoTypes = {
        button: PropTypes.object,
        name: PropTypes.string,
}

export default RenameButton